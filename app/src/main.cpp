/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <opencv2/imgproc.hpp>
#include <se/supereight.hpp>
#include <thread>
#include <torch/torch.h>

#include "config.hpp"
#include "gui.hpp"
#include "gs/gaussian.cuh"
#include "gs/gaussian_utils.cuh"
#include "reader.hpp"
#include "se/common/filesystem.hpp"
#include "se/common/system_utils.hpp"


#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress(double percentage)
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    if (val == 100) {
        printf("\n");
    }
    fflush(stdout);
}


int main(int argc, char** argv)
{
    try {
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " YAML_FILE\n";
            return 2;
        }

        auto mem_before = gs::getGPUMemoryUsage();

        // ========= Config & I/O INITIALIZATION  =========
        const std::string config_filename = argv[1];
        const se::Config<se::TSDFColDataConfig, se::PinholeCameraConfig> config(config_filename);

        // Create the mesh output directory
        if (!config.app.mesh_path.empty()) {
            stdfs::create_directories(config.app.mesh_path);
        }
        if (!config.app.slice_path.empty()) {
            stdfs::create_directories(config.app.slice_path);
        }
        if (!config.app.structure_path.empty()) {
            stdfs::create_directories(config.app.structure_path);
        }

        // Setup log stream
        std::ofstream log_file_stream;
        log_file_stream.open(config.app.log_file);
        se::perfstats.setFilestream(&log_file_stream);

        // Setup input images
        const Eigen::Vector2i input_img_res(config.sensor.width, config.sensor.height);
        se::Image<float> input_depth_img(input_img_res.x(), input_img_res.y());
        se::Image<se::rgb_t> input_colour_img(input_img_res.x(), input_img_res.y(), {0, 0, 0});

        // ========= Map INITIALIZATION  =========
        // Setup the single-res TSDF map w/ default block size of 8 voxels
        se::TSDFColMap<se::Res::Single> map(config.map, config.data);

        // ========= Sensor INITIALIZATION  =========
        // Create a pinhole camera
        const se::PinholeCamera sensor(config.sensor);

        // ========= Gaussian Model INITIALIZATION  =========
        auto optimParams = gs::param::read_optim_params_from_json(config.app.optim_params_path);
        gs::GaussianModel gs_model = gs::GaussianModel(optimParams, config.app.ply_path);
        std::vector<gs::Camera> gs_cam_list;
        std::vector<torch::Tensor> gt_img_list;

        // Write cfg_args file
        const std::string cfg_args_file = stdfs::path(config.app.ply_path).parent_path() / "cfg_args";
        std::ofstream fs(cfg_args_file, std::ios::out);
        if (!fs.good()) {
            std::cerr << "Failed to open cfg_args for writing!" << std::endl;
        }

        std::string image_folder_name;
        if (se::reader_type_to_string(config.reader.reader_type) == "ScanNetpp") {
            image_folder_name = "undistorted_images_2";
        }
        else if (se::reader_type_to_string(config.reader.reader_type) == "Replica") {
            image_folder_name = "results";
        }

        fs << "Namespace("
           << "eval=True, "
           << "images="
           << "\"" << image_folder_name << "\", "
           << "model_path=" << stdfs::path(config.app.ply_path).parent_path() << ", "
           << "resolution=-1, "
           << "sh_degree=" << gs_model.optimParams.sh_degree << ", "
           << "source_path="
           << "\"" << config.reader.sequence_path << "\", "
           << "white_background=False)";
        fs.close();

        // ========= GUI INITIALIZATION  =========
        gs::DataQueue data_queue;
        std::atomic<bool> stop_signal(false);
        GUI gs_gui(data_queue, stop_signal, input_img_res.x(), input_img_res.y());
        std::thread gui_thread([&]() { gs_gui.run(); });

        // ========= READER INITIALIZATION  =========
        se::Reader* reader = nullptr;
        reader = se::create_reader(config.reader);

        if (reader == nullptr) {
            return EXIT_FAILURE;
        }

        Eigen::Matrix4f T_WB = Eigen::Matrix4f::Identity(); //< Body to world transformation
        Eigen::Matrix4f T_BS = sensor.T_BS;                 //< Sensor to body transformation
        Eigen::Matrix4f T_WS = T_WB * T_BS;                 //< Sensor to world transformation

        // ========= Integrator INITIALIZATION  =========
        int frame = 0;
        float mean_fps = 0.0f;
        while (frame != config.app.max_frames) {
            se::perfstats.setIter(frame++);

            TICK("total")
            TICK("read")
            se::ReaderStatus read_ok = se::ReaderStatus::ok;
            if (config.app.enable_ground_truth || frame == 1) {
                read_ok = reader->nextData(input_depth_img, input_colour_img, T_WB);
                T_WS = T_WB * T_BS;
            }
            else {
                read_ok = reader->nextData(input_depth_img, input_colour_img);
            }
            if (read_ok != se::ReaderStatus::ok) {
                break;
            }
            TOCK("read")

            TICK("integration")
            double s = PerfStats::getTime();
            if (frame % config.app.integration_rate == 0) {
                se::integrator::integrate(map, gs_model, gs_cam_list, gt_img_list, data_queue, input_depth_img, input_colour_img, sensor, T_WS, frame);
            }
            double e = PerfStats::getTime();
            mean_fps += (1 / (e - s));
            TOCK("integration")
            TOCK("total")

            const bool last_frame = frame == config.app.max_frames || static_cast<size_t>(frame) == reader->numFrames();
            if (last_frame) {
                double s = PerfStats::getTime();

                // Refresh GUI
                gs::DataPacket data_packet;
                data_packet.num_kf = gt_img_list.size();
                data_packet.rgb = cv::Mat(input_img_res.y(), input_img_res.x(), CV_8UC3, cv::Scalar(0, 0, 0));
                data_packet.depth = cv::Mat(input_img_res.y(), input_img_res.x(), CV_8UC3, cv::Scalar(0, 0, 0));
                data_packet.rendered_rgb = cv::Mat(input_img_res.y(), input_img_res.x(), CV_8UC3, cv::Scalar(0, 0, 0));
                data_queue.push(data_packet);

                // Global optimizaiton of reconstructed GS map (offline)
                auto lambda = gs_model.optimParams.lambda_dssim;
                auto iters = gs_model.optimParams.global_iters;
                for (int it = 0; it < iters; it++) {
                    std::vector<int> indices = gs::get_random_indices(gt_img_list.size());
                    for (int i = 0; i < indices.size(); i++) {
                        auto cur_gt_img = gt_img_list[indices[i]];
                        auto cur_gs_cam = gs_cam_list[indices[i]];

                        auto [image, viewspace_point_tensor, visibility_filter, radii] = gs::render(cur_gs_cam, gs_model);

                        // Loss Computations
                        auto l1_loss = gs::l1_loss(image, cur_gt_img);
                        auto ssim_loss = gs::ssim(image, cur_gt_img, gs::conv_window, gs::window_size, gs::channel);
                        auto loss = (1.f - lambda) * l1_loss + lambda * (1.f - ssim_loss);

                        // Optimization
                        loss.backward();
                        gs_model.optimizer->step();
                        gs_model.optimizer->zero_grad(true);

                        if (i == indices.size() - 1) {
                            auto rendered_img_tensor = image.detach().permute({1, 2, 0}).contiguous().to(torch::kCPU);
                            rendered_img_tensor = rendered_img_tensor.mul(255).clamp(0, 255).to(torch::kU8);
                            auto cv_rendered_img = cv::Mat(image.size(1), image.size(2), CV_8UC3, rendered_img_tensor.data_ptr());
                            data_packet.rendered_rgb = cv_rendered_img;
                            data_packet.global_iter = it + 1;
                            data_queue.push(data_packet);
                        }
                    }
                }
                torch::cuda::synchronize();
                double e = PerfStats::getTime();

                // Get GPU memory usage
                auto mem_after = gs::getGPUMemoryUsage();

                std::cout << "Avg. fps: " << mean_fps / frame << std::endl;
                std::cout << "Global opt. time: " << e - s << " s" << std::endl;
                std::cout << "GPU memory usage: " << mem_after - mem_before << " MB" << std::endl;
                std::cout << "#Keyframes: " << gt_img_list.size() << std::endl;

                // Write mapping statistics to a file
                const std::string stats_file = stdfs::path(config.app.ply_path).parent_path() / "stats";
                std::ofstream fs(stats_file, std::ios::out);
                if (!fs.good()) {
                    std::cerr << "Failed to open stats for writing!" << std::endl;
                }
                fs << "Avg. fps: " << mean_fps / frame << " Hz\n"
                   << "Global opt. time: " << e - s << " s\n"
                   << "GPU memory usage: " << mem_after - mem_before << " MB\n"
                   << "#Keyframes: " << gt_img_list.size() << "\n";

                gs_model.Save_ply(gs_model.output_path, frame, true);
            }

            // Save mesh if enabled
            if ((config.app.meshing_rate > 0 && frame % config.app.meshing_rate == 0) || last_frame) {
                if (!config.app.mesh_path.empty()) {
                    map.saveMesh(config.app.mesh_path + "/mesh_" + std::to_string(frame) + ".ply");
                }
                if (!config.app.slice_path.empty()) {
                    map.saveFieldSlices(config.app.slice_path + "/slice_x_" + std::to_string(frame) + ".vtk",
                                        config.app.slice_path + "/slice_y_" + std::to_string(frame) + ".vtk",
                                        config.app.slice_path + "/slice_z_" + std::to_string(frame) + ".vtk",
                                        se::math::to_translation(T_WS));
                }
                if (!config.app.structure_path.empty()) {
                    map.saveStructure(config.app.structure_path + "/struct_" + std::to_string(frame) + ".ply");
                }
            }

            se::perfstats.sample("memory usage", se::system::memory_usage_self() / 1024.0 / 1024.0, PerfStats::MEMORY);
            se::perfstats.writeToFilestream();
            printProgress(static_cast<double>(frame) / (static_cast<double>(reader->numFrames()) - 1));
        }

        stop_signal.store(true);
        gui_thread.join();

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
