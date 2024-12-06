/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: MIT
 */

#include "reader_replica.hpp"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <set>

#include "se/common/filesystem.hpp"
#include "se/common/image_utils.hpp"


/** A timestamped ground truth pose and its associated depth and RGB images.
 */
struct ReplicaPoseEntry {
    double timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    std::string depth_filename;
    std::string rgb_filename;

    /** Initialize an invalid ReplicaPoseEntry.
     */
    ReplicaPoseEntry()
    {
    }

    ReplicaPoseEntry(const double t, const Eigen::Vector3f& p, const Eigen::Quaternionf& o, const std::string& df, const std::string& rf) :
            timestamp(t), position(p), orientation(o), depth_filename(df), rgb_filename(rf)
    {
    }

    /** Initialize using a single-line string from a Replica traj.txt.
     * depth_filename and rgb_filename will not be initialized.
     * \warning No error checking is performed in this function, it should be
     * performed by the caller.
     */
    ReplicaPoseEntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ' ', true);

        auto currentTime = std::chrono::system_clock::now();
        auto duration = currentTime.time_since_epoch();
        timestamp = std::chrono::duration<double>(duration).count();

        Eigen::Matrix4f C2W;
        C2W << std::stof(columns[0]), std::stof(columns[1]), std::stof(columns[2]), std::stof(columns[3]), std::stof(columns[4]), std::stof(columns[5]), std::stof(columns[6]), std::stof(columns[7]),
            std::stof(columns[8]), std::stof(columns[9]), std::stof(columns[10]), std::stof(columns[11]), std::stof(columns[12]), std::stof(columns[13]), std::stof(columns[14]),
            std::stof(columns[15]);

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


/** Read a Replica traj.txt into an std::vector of ReplicaPoseEntry.
 * Return an empty std::vector if the file was not in the correct format.
 */
std::vector<ReplicaPoseEntry> read_replica_ground_truth(const std::string& filename)
{
    std::vector<ReplicaPoseEntry> poses;
    std::ifstream fs(filename, std::ios::in);
    if (!fs.good()) {
        std::cerr << "Error: Could not read ground truth file " << filename << "\n";
        return poses;
    }

    // Read all data lines
    for (std::string line; std::getline(fs, line);) {
        // Add the pose
        poses.emplace_back(line);
    }
    if (poses.empty()) {
        std::cerr << "Error: Empty ground truth file " << filename << "\n";
    }
    return poses;
}


/** Generate a ground truth file from poses and write it in a temporary file.
 */
std::string write_ground_truth_tmp(const std::vector<ReplicaPoseEntry>& poses)
{
    // Open a temporary file
    const std::string tmp_filename = stdfs::temp_directory_path() / "replica_gt.txt";
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


// ReplicaReader implementation
constexpr float se::ReplicaReader::replica_inverse_scale_;

se::ReplicaReader::ReplicaReader(const se::ReaderConfig& c) : se::Reader(c)
{
    inverse_scale_ = (c.inverse_scale != 0) ? c.inverse_scale : replica_inverse_scale_;

    // Ensure sequence_path_ refers to a valid Replica directory structure. Only depth data is
    // required to exist.
    if (!stdfs::is_directory(sequence_path_) || !stdfs::is_directory(sequence_path_ + "/results") || !stdfs::is_regular_file(sequence_path_ + "/results/depth000000.png")) {
        std::cerr << "Error: The Replica sequence path must be a directory that contains"
                  << " a results/ subdirectory and results/depth*.png files\n";
        status_ = se::ReaderStatus::error;
        return;
    }

    // Get the filenames of the depth and RGB images
    std::vector<std::string> allFileNames;
    cv::glob(sequence_path_ + "/results", allFileNames);
    // Separate RGB and depth images
    for (const auto& fileName : allFileNames) {
        // Check if the file is an RGB or depth image based on the prefix
        if (fileName.find("frame") != std::string::npos) {
            rgb_filenames_.push_back(fileName);
        }
        else if (fileName.find("depth") != std::string::npos) {
            depth_filenames_.push_back(fileName);
        }
    }

    if (depth_filenames_.empty()) {
        std::cerr << "Error: No Replica depth images found in results/\n";
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
        std::cerr << "Warning: No Replica colour images found in results/\n";
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

    // Read the ground truth file if needed
    if (!ground_truth_file_.empty()) {
        std::vector<ReplicaPoseEntry> gt_poses = read_replica_ground_truth(ground_truth_file_);
        if (gt_poses.empty()) {
            status_ = se::ReaderStatus::error;
            return;
        }
        for (size_t i = 0; i < gt_poses.size(); i++) {
            gt_poses[i].depth_filename = depth_filenames_[i];
            gt_poses[i].rgb_filename = rgb_filenames_[i];
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

    num_frames_ = depth_filenames_.size();
    has_colour_ = !rgb_filenames_.empty();
}


void se::ReplicaReader::restart()
{
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
    }
}


std::string se::ReplicaReader::name() const
{
    return std::string("ReplicaReader");
}


se::ReaderStatus se::ReplicaReader::nextDepth(se::Image<float>& depth_image)
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


se::ReaderStatus se::ReplicaReader::nextColour(se::Image<rgb_t>& colour_image)
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