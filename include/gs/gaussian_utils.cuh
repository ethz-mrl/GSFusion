/*
 * SPDX-FileCopyrightText: 2023 Janusch Patas
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * All rights reserved. Some of the code is derived from 3D Gaussian Splatting for Real-Time Radiance Field Rendering software by Inria and MPII.
 */

#ifndef GS_GAUSSIAN_UTIL_HPP
#define GS_GAUSSIAN_UTIL_HPP

#include <Eigen/Dense>
#include <climits>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <nvml.h>
#include <opencv2/opencv.hpp>
#include <queue>
#include <random>
#include <tinyply.h>
#include <torch/torch.h>
#include <filesystem>

using Slice = torch::indexing::Slice;

namespace gs {
struct DataPacket {
    int ID = -1;
    int global_iter = -1;
    int num_kf;
    int num_splats;
    float fps;
    cv::Mat rgb;
    cv::Mat depth;
    cv::Mat rendered_rgb;
};


class DataQueue {
    public:
    void push(DataPacket data)
    {
        std::lock_guard<std::mutex> lock(_mtx);
        _queue.push(std::move(data));
        _cv.notify_one(); // Notify one waiting thread
    }

    DataPacket pop()
    {
        std::unique_lock<std::mutex> lock(_mtx);
        _cv.wait(lock, [this]() { return !_queue.empty(); }); // Wait until queue is not empty
        DataPacket data = std::move(_queue.front());
        _queue.pop();
        return data;
    }

    int getSize()
    {
        return _queue.size();
    }

    private:
    std::queue<DataPacket> _queue;
    std::mutex _mtx;
    std::condition_variable _cv;
};

// 1D Gaussian kernel
static inline torch::Tensor gaussian(int window_size, float sigma)
{
    torch::Tensor gauss = torch::empty(window_size);
    for (int x = 0; x < window_size; ++x) {
        gauss[x] = std::exp(-(std::pow(static_cast<float>(x - std::floor(window_size / 2.f)), 2)) / (2.f * sigma * sigma));
    }
    return gauss / gauss.sum();
}

static inline torch::Tensor create_window(int window_size, int channel)
{
    auto _1D_window = gaussian(window_size, 1.5).unsqueeze(1);
    auto _2D_window = _1D_window.mm(_1D_window.t()).unsqueeze(0).unsqueeze(0);
    return _2D_window.expand({channel, 1, window_size, window_size}).contiguous();
}

const int window_size = 11;
const int channel = 3;
const auto conv_window = create_window(window_size, channel).to(torch::kFloat32).to(torch::kCUDA, true);

static const double C0 = 0.28209479177387814;
static const double C1 = 0.4886025119029199;
static std::vector<double> C2 = {1.0925484305920792, -1.0925484305920792, 0.31539156525252005, -1.0925484305920792, 0.5462742152960396};
static std::vector<double> C3 = {-0.5900435899266435, 2.890611442640554, -0.4570457994644658, 0.3731763325901154, -0.4570457994644658, 1.445305721320277, -0.5900435899266435};
static std::vector<double> C4 =
    {2.5033429417967046, -1.7701307697799304, 0.9461746957575601, -0.6690465435572892, 0.10578554691520431, -0.6690465435572892, 0.47308734787878004, -1.7701307697799304, 0.6258357354491761};

static inline torch::Tensor RGB2SH(const torch::Tensor& rgb)
{
    return (rgb - 0.5f) / static_cast<float>(C0);
}

struct Point {
    float x;
    float y;
    float z;
};

struct Color {
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

struct Camera {
    int width;
    int height;
    float fov_x;
    float fov_y;
    torch::Tensor T_W2C;
    torch::Tensor full_proj_matrix;
    torch::Tensor cam_center;
};

/**
     * Calculates the inverse sigmoid of a tensor element-wise.
     *
     * @param x The input tensor.
     * @return The tensor with the inverse sigmoid of each element.
     */
static inline torch::Tensor inverse_sigmoid(torch::Tensor x)
{
    return torch::log(x / (1 - x));
}

/**
     * @brief Strips the lower diagonal elements of a 3x3 matrix and returns them as a 1D tensor of size (N, 6).
     *
     * @param L A 3D tensor of size (N, 3, 3) representing a batch of 3x3 matrices.
     * @return A 2D tensor of size (N, 6) representing the upper diagonal elements of each matrix in L.
     */
static inline torch::Tensor strip_lowerdiag(torch::Tensor L)
{
    torch::Tensor uncertainty = torch::zeros({L.size(0), 6}, torch::dtype(torch::kFloat).device(torch::kCUDA));

    uncertainty.index_put_({Slice(), 0}, L.index({Slice(), 0, 0}));
    uncertainty.index_put_({Slice(), 1}, L.index({Slice(), 0, 1}));
    uncertainty.index_put_({Slice(), 2}, L.index({Slice(), 0, 2}));
    uncertainty.index_put_({Slice(), 3}, L.index({Slice(), 1, 1}));
    uncertainty.index_put_({Slice(), 4}, L.index({Slice(), 1, 2}));
    uncertainty.index_put_({Slice(), 5}, L.index({Slice(), 2, 2}));
    return uncertainty;
}

/**
     * @brief Strips the symmetric part of a tensor.
     *
     * This function takes an input tensor and returns the tensor with the symmetric part removed.
     *
     * @param sym The input tensor.
     * @return The tensor with the symmetric part removed.
     */
static inline torch::Tensor strip_symmetric(torch::Tensor sym)
{
    return strip_lowerdiag(sym);
}

/**
     * @brief Builds a rotation matrix from a tensor of quaternions.
     *
     * @param r Tensor of quaternions with shape (N, 4).
     * @return Tensor of rotation matrices with shape (N, 3, 3).
     */
static inline torch::Tensor build_rotation(torch::Tensor r)
{
    torch::Tensor norm = torch::sqrt(torch::sum(r.pow(2), 1));
    torch::Tensor q = r / norm.unsqueeze(-1);

    torch::Tensor R = torch::zeros({q.size(0), 3, 3}, torch::device(torch::kCUDA));
    torch::Tensor r0 = q.index({Slice(), 0});
    torch::Tensor x = q.index({Slice(), 1});
    torch::Tensor y = q.index({Slice(), 2});
    torch::Tensor z = q.index({Slice(), 3});

    R.index_put_({Slice(), 0, 0}, 1 - 2 * (y * y + z * z));
    R.index_put_({Slice(), 0, 1}, 2 * (x * y - r0 * z));
    R.index_put_({Slice(), 0, 2}, 2 * (x * z + r0 * y));
    R.index_put_({Slice(), 1, 0}, 2 * (x * y + r0 * z));
    R.index_put_({Slice(), 1, 1}, 1 - 2 * (x * x + z * z));
    R.index_put_({Slice(), 1, 2}, 2 * (y * z - r0 * x));
    R.index_put_({Slice(), 2, 0}, 2 * (x * z - r0 * y));
    R.index_put_({Slice(), 2, 1}, 2 * (y * z + r0 * x));
    R.index_put_({Slice(), 2, 2}, 1 - 2 * (x * x + y * y));
    return R;
}

/**
     * Builds a scaling-rotation matrix from the given scaling and rotation tensors.
     *
     * @param s The scaling tensor of shape (N, 3) where N is the number of scaling factors.
     * @param r The rotation tensor of shape (N, 4) where N is the number of rotation angles.
     * @return The scaling-rotation matrix of shape (N, 3, 3) where N is the number of scaling-rotation matrices.
     */
static inline torch::Tensor build_scaling_rotation(torch::Tensor s, torch::Tensor r)
{
    torch::Tensor L = torch::zeros({s.size(0), 3, 3}, torch::dtype(torch::kFloat).device(torch::kCUDA));
    torch::Tensor R = build_rotation(r);

    L.index_put_({Slice(), 0, 0}, s.index({Slice(), 0}));
    L.index_put_({Slice(), 1, 1}, s.index({Slice(), 1}));
    L.index_put_({Slice(), 2, 2}, s.index({Slice(), 2}));

    L = R.matmul(L);
    return L;
}

static inline torch::Tensor getProjectionMatrix(int W, int H, float fx, float fy, float cx, float cy)
{
    Eigen::Matrix4f P = Eigen::Matrix4f::Zero();
    P(0, 0) = 2 * fx / W;
    P(1, 1) = 2 * fy / H;
    P(0, 2) = 2 * (cx / W) - 1;
    P(1, 2) = 2 * (cy / H) - 1;
    P(2, 2) = 0;
    P(2, 3) = 0;
    P(3, 2) = 1;

    // create torch::Tensor from Eigen::Matrix
    auto PTensor = torch::from_blob(P.data(), {4, 4}, torch::kFloat);
    return PTensor.clone();
}

static inline void Write_output_ply(const std::filesystem::path& file_path, const std::vector<torch::Tensor>& tensors, const std::vector<std::string>& attribute_names)
{
    tinyply::PlyFile plyFile;

    size_t attribute_offset = 0; // An offset to track the attribute names

    for (size_t i = 0; i < tensors.size(); ++i) {
        // Calculate the number of columns in the tensor.
        size_t columns = tensors[i].size(1);

        std::vector<std::string> current_attributes;
        for (size_t j = 0; j < columns; ++j) {
            current_attributes.push_back(attribute_names[attribute_offset + j]);
        }

        plyFile.add_properties_to_element(
            "vertex", current_attributes, tinyply::Type::FLOAT32, tensors[i].size(0), reinterpret_cast<uint8_t*>(tensors[i].data_ptr<float>()), tinyply::Type::INVALID, 0);

        attribute_offset += columns; // Increase the offset for the next tensor.
    }

    std::filebuf fb;
    fb.open(file_path, std::ios::out | std::ios::binary);
    std::ostream outputStream(&fb);
    plyFile.write(outputStream, true); // 'true' for binary format
}

static inline float psnr_metric(const torch::Tensor& rendered_img, const torch::Tensor& gt_img)
{
    torch::Tensor squared_diff = (rendered_img - gt_img).pow(2);
    torch::Tensor mse_val = squared_diff.view({rendered_img.size(0), -1}).mean(1, true);
    return (20.f * torch::log10(1.0 / mse_val.sqrt())).mean().item<float>();
}

static inline std::vector<int> get_random_indices(int max_index)
{
    std::vector<int> indices(max_index);
    std::iota(indices.begin(), indices.end(), 0);
    // Shuffle the vector
    std::random_device rd;
    std::shuffle(indices.begin(), indices.end(), std::default_random_engine(rd()));
    std::reverse(indices.begin(), indices.end());
    return indices;
}

// Function to get the memory usage of the GPU
static inline auto getGPUMemoryUsage(int gpu_id = 0)
{
    nvmlReturn_t result;
    nvmlDevice_t device;
    nvmlMemory_t memory;

    // Initialize NVML library
    result = nvmlInit();
    if (NVML_SUCCESS != result) {
        std::cerr << "Failed to initialize NVML: " << nvmlErrorString(result) << "\n";
        return ULLONG_MAX;
    }

    // Get the GPU handle
    result = nvmlDeviceGetHandleByIndex(gpu_id, &device);
    if (NVML_SUCCESS != result) {
        std::cerr << "Failed to get handle for device " << std::to_string(gpu_id) << ": " << nvmlErrorString(result) << "\n";
        nvmlShutdown();
        return ULLONG_MAX;
    }

    // Get memory information
    result = nvmlDeviceGetMemoryInfo(device, &memory);
    if (NVML_SUCCESS != result) {
        std::cerr << "Failed to get memory info: " << nvmlErrorString(result) << "\n";
        nvmlShutdown();
        return ULLONG_MAX;
    }

    auto used_gpu_memory = memory.used / (1024 * 1024);

    // Shutdown NVML library
    nvmlShutdown();

    return used_gpu_memory;
}
} // namespace gs

#endif // GS_GAUSSIAN_UTIL_HPP