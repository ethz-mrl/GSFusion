/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: MIT
 */

#include "reader_scannetpp.hpp"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <set>

#include "se/common/filesystem.hpp"
#include "se/common/image_utils.hpp"


/** A timestamped ground truth pose and its associated depth and RGB images.
 */
struct ScanNetppPoseEntry {
    double timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    std::string depth_filename;
    std::string rgb_filename;

    /** Initialize an invalid ScanNetppPoseEntry.
     */
    ScanNetppPoseEntry()
    {
    }

    ScanNetppPoseEntry(const double t, const Eigen::Vector3f& p, const Eigen::Quaternionf& o, const std::string& df, const std::string& rf) :
            timestamp(t), position(p), orientation(o), depth_filename(df), rgb_filename(rf)
    {
    }

    /** Initialize using a transform_matrix from a ScanNet++ transforms_undistorted_2.json.
     * \warning No error checking is performed in this function, it should be
     * performed by the caller.
     */
    ScanNetppPoseEntry(const Eigen::Matrix4f& C2W)
    {
        auto currentTime = std::chrono::system_clock::now();
        auto duration = currentTime.time_since_epoch();
        timestamp = std::chrono::duration<double>(duration).count();
        position = C2W.block<3, 1>(0, 3);
        orientation = C2W.block<3, 3>(0, 0);
    }

    /** Return a single-line string representation of the ground truth pose.
     * It can be used to write it to a ground truth file that is understood by
     * supereight.
     */
    std::string string() const
    {
        const std::string s = std::to_string(timestamp) + " " + rgb_filename + " " + depth_filename + " " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " "
            + std::to_string(position.z()) + " " + std::to_string(orientation.x()) + " " + std::to_string(orientation.y()) + " " + std::to_string(orientation.z()) + " "
            + std::to_string(orientation.w());
        return s;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


nlohmann::json read_json_file(const std::string& filename)
{
    stdfs::path json_path = filename;
    if (!stdfs::exists(json_path)) {
        throw std::runtime_error("Error: " + json_path.string() + " does not exist!");
    }

    std::ifstream file(json_path);
    if (!file.is_open()) {
        throw std::runtime_error("Error: " + json_path.string() + " could not be opened!");
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonString = buffer.str();
    file.close(); // Explicitly close the file

    // Parse the JSON string
    nlohmann::json json = nlohmann::json::parse(jsonString);
    return json;
}


/** Read a ScanNet++ transforms_undistorted_2.json into an std::vector of ScanNetppPoseEntry.
 * Return an empty std::vector if the file was not in the correct format.
 */
std::vector<ScanNetppPoseEntry> read_scannetpp_ground_truth(const std::string& gt_filename, const std::string& data_root)
{
    std::vector<ScanNetppPoseEntry> poses;
    auto gt_json = read_json_file(gt_filename);
    auto filename_json = read_json_file(data_root + "/train_test_lists.json");

    std::vector<std::string> file_path;
    for (size_t i = 0; i < gt_json["frames"].size(); i++) {
        file_path.push_back(gt_json["frames"][i]["file_path"]);
    }
    for (auto name : filename_json["train"]) {
        auto it = std::find(file_path.begin(), file_path.end(), name);
        auto index = std::distance(file_path.begin(), it);
        std::string new_name = file_path[index];
        new_name.replace(new_name.find(".JPG"), 4, ".png");

        /* Coordinate convensions
        ScanNet++ uses the OpenGL/Blender (and original NeRF) coordinate convention for cameras. 
        +X is right, +Y is up, and +Z is pointing back and away from the camera. -Z is the look-at direction. 
        */
        auto transform_matrix = gt_json["frames"][index]["transform_matrix"];
        Eigen::Matrix4f P = Eigen::Matrix4f::Identity();
        P(1, 1) = -1;
        P(2, 2) = -1;
        Eigen::Matrix4f C2W;
        C2W << transform_matrix[0][0], transform_matrix[0][1], transform_matrix[0][2], transform_matrix[0][3], transform_matrix[1][0], transform_matrix[1][1], transform_matrix[1][2],
            transform_matrix[1][3], transform_matrix[2][0], transform_matrix[2][1], transform_matrix[2][2], transform_matrix[2][3], transform_matrix[3][0], transform_matrix[3][1],
            transform_matrix[3][2], transform_matrix[3][3];
        poses.emplace_back(C2W * P);
        poses.back().depth_filename = data_root + "/undistorted_depths_2/" + new_name;
        poses.back().rgb_filename = data_root + "/undistorted_images_2/" + file_path[index];
    }

    if (poses.empty()) {
        std::cerr << "Error: Empty ground truth file " << gt_filename << "\n";
    }
    return poses;
}


/** Generate a ground truth file from poses and write it in a temporary file.
 */
std::string write_ground_truth_tmp(const std::vector<ScanNetppPoseEntry>& poses)
{
    // Open a temporary file
    const std::string tmp_filename = stdfs::temp_directory_path() / "scannetpp_gt.txt";
    std::ofstream fs(tmp_filename, std::ios::out);
    if (!fs.good()) {
        std::cerr << "Error: Could not write associated ground truth file " << tmp_filename << "\n";
        return "";
    }
    // Write the header
    fs << "# Association of rgb images, depth images and ground truth poses\n";
    fs << "# ID timestamp rgb_filename depth_filename tx ty tz qx qy qz qw\n";
    // Write each of the associated poses
    for (size_t i = 0; i < poses.size(); ++i) {
        fs << std::setw(6) << std::setfill('0') << i << " " << poses[i].string() << "\n";
    }
    return tmp_filename;
}


// ScanNetppReader implementation
constexpr float se::ScanNetppReader::scannetpp_inverse_scale_;

se::ScanNetppReader::ScanNetppReader(const se::ReaderConfig& c) : se::Reader(c)
{
    inverse_scale_ = (c.inverse_scale != 0) ? c.inverse_scale : scannetpp_inverse_scale_;

    // Ensure sequence_path_ refers to a valid ScanNet++ directory structure. Only depth data is
    // required to exist.
    if (!stdfs::is_directory(sequence_path_) || !stdfs::is_directory(sequence_path_ + "/undistorted_depths_2") || !stdfs::exists(sequence_path_ + "/train_test_lists.json")) {
        std::cerr << "Error: The ScanNet++ sequence path must be a directory that contains"
                  << " a undistorted_depths_2/ subdirectory and a train_test_lists.json file\n";
        status_ = se::ReaderStatus::error;
        return;
    }

    if (!stdfs::is_directory(sequence_path_ + "/undistorted_images_2")) {
        std::cerr << "Warning: No undistorted_images_2/ subdirectory in the provided sequence path\n";
    }

    // Read the ground truth file if needed
    if (!ground_truth_file_.empty()) {
        std::vector<ScanNetppPoseEntry> gt_poses = read_scannetpp_ground_truth(ground_truth_file_, sequence_path_);
        if (gt_poses.empty()) {
            status_ = se::ReaderStatus::error;
            return;
        }

        for (size_t i = 0; i < gt_poses.size(); i++) {
            depth_filenames_.push_back(gt_poses[i].depth_filename);
            rgb_filenames_.push_back(gt_poses[i].rgb_filename);
        }

        // Generate the associated ground truth file
        const std::string generated_filename = write_ground_truth_tmp(gt_poses);
        // Close the original ground truth file and open the generated one
        ground_truth_fs_.close();
        ground_truth_fs_.open(generated_filename, std::ios::in);
        if (!ground_truth_fs_.good()) {
            std::cerr << "Error: Could not read generated ground truth file " << generated_filename << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
    }

    if (depth_filenames_.empty()) {
        std::cerr << "Error: No ScanNet++ depth images found in undistorted_depths_2/\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    // Set the depth image resolution to that of the first depth image.
    const std::string first_depth_filename = depth_filenames_[0];
    cv::Mat image_data = cv::imread(first_depth_filename.c_str(), cv::IMREAD_UNCHANGED);
    if (image_data.empty()) {
        std::cerr << "Error: Could not read depth image " << first_depth_filename << "\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    depth_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);

    if (rgb_filenames_.empty()) {
        std::cerr << "Warning: No ScanNet++ colour images found in undistorted_images_2/\n";
    }
    else {
        // Set the colour image resolution to that of the first colour image.
        const std::string first_rgb_filename = rgb_filenames_[0];
        cv::Mat image_data = cv::imread(first_rgb_filename.c_str(), cv::IMREAD_COLOR);
        if (image_data.data == NULL) {
            std::cerr << "Error: Could not read colour image " << first_rgb_filename << "\n";
            rgb_filenames_.clear();
        }
        else {
            colour_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);
        }
    }

    num_frames_ = depth_filenames_.size();
    has_colour_ = !rgb_filenames_.empty();
}


void se::ScanNetppReader::restart()
{
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
    }
}


std::string se::ScanNetppReader::name() const
{
    return std::string("ScanNetppReader");
}


se::ReaderStatus se::ScanNetppReader::nextDepth(se::Image<float>& depth_image)
{
    if (frame_ >= num_frames_) {
        return se::ReaderStatus::error;
    }

    // Read the image data.
    const std::string filename = depth_filenames_[frame_];
    cv::Mat image_data = cv::imread(filename.c_str(), cv::IMREAD_UNCHANGED);
    if (image_data.empty()) {
        return se::ReaderStatus::error;
    }

    cv::Mat depth_data;
    image_data.convertTo(depth_data, CV_32FC1, inverse_scale_);

    assert(depth_image_res_.x() == static_cast<int>(image_data.cols));
    assert(depth_image_res_.y() == static_cast<int>(image_data.rows));
    // Resize the output image if needed.
    if ((depth_image.width() != depth_image_res_.x()) || (depth_image.height() != depth_image_res_.y())) {
        depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
    }

    cv::Mat wrapper_mat(depth_data.rows, depth_data.cols, CV_32FC1, depth_image.data());
    depth_data.copyTo(wrapper_mat);
    return se::ReaderStatus::ok;
}


se::ReaderStatus se::ScanNetppReader::nextColour(se::Image<rgb_t>& colour_image)
{
    if (frame_ >= num_frames_ || rgb_filenames_.empty()) {
        return se::ReaderStatus::error;
    }

    // Read the image data.
    const std::string filename = rgb_filenames_[frame_];
    cv::Mat image_data = cv::imread(filename.c_str(), cv::IMREAD_COLOR);
    if (image_data.empty()) {
        return se::ReaderStatus::error;
    }

    cv::Mat colour_data;
    cv::cvtColor(image_data, colour_data, cv::COLOR_BGR2RGB);

    assert(colour_image_res_.x() == static_cast<int>(colour_data.cols));
    assert(colour_image_res_.y() == static_cast<int>(colour_data.rows));
    // Resize the output image if needed.
    if ((colour_image.width() != colour_image_res_.x()) || (colour_image.height() != colour_image_res_.y())) {
        colour_image = se::Image<rgb_t>(colour_image_res_.x(), colour_image_res_.y());
    }

    cv::Mat wrapper_mat(colour_data.rows, colour_data.cols, CV_8UC3, colour_image.data());
    colour_data.copyTo(wrapper_mat);
    return se::ReaderStatus::ok;
}