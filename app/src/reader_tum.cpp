/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_tum.hpp"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <algorithm>
#include <cassert>
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


/** The filename of a depth or RGB image and its timestamp.
  */
struct TUMImageEntry {
    double timestamp;
    std::string filename;

    /** Initialize an invalid TUMImageEntry.
      */
    TUMImageEntry() : timestamp(NAN)
    {
    }

    /** Initialize using a single-line string from a TUM depth.txt or rgb.txt.
      * \warning No error checking is performed in this function, it should be
      * performed by the caller.
      */
    TUMImageEntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ' ', true);
        timestamp = std::stod(columns[0]);
        filename = columns[1];
    }
};


/** A timestamped ground truth pose and its associated depth and RGB images.
  */
struct TUMPoseEntry {
    double timestamp;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    std::string depth_filename;
    std::string rgb_filename;

    /** Initialize an invalid TUMPoseEntry.
      */
    TUMPoseEntry() : timestamp(NAN)
    {
    }

    TUMPoseEntry(const double t, const Eigen::Vector3f& p, const Eigen::Quaternionf& o, const std::string& df, const std::string& rf) :
            timestamp(t), position(p), orientation(o), depth_filename(df), rgb_filename(rf)
    {
    }

    /** Initialize using a single-line string from a TUM groundtruth.txt.
      * depth_filename and rgb_filename will not be initialized.
      * \warning No error checking is performed in this function, it should be
      * performed by the caller.
      */
    TUMPoseEntry(const std::string& s)
    {
        const std::vector<std::string> columns = se::str_utils::split_str(s, ' ', true);
        timestamp = std::stod(columns[0]);
        position = Eigen::Vector3f(std::stof(columns[1]), std::stof(columns[2]), std::stof(columns[3]));
        orientation = Eigen::Quaternionf(std::stof(columns[7]), std::stof(columns[4]), std::stof(columns[5]), std::stof(columns[6]));
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


/** Read a TUM depth.txt or rgb.txt into an std::vector of TUMImageEntry.
  * Return an empty std::vector if the file was not in the correct format.
  */
std::vector<TUMImageEntry> read_tum_image_list(const std::string& filename)
{
    std::vector<TUMImageEntry> images;
    std::ifstream fs(filename, std::ios::in);
    if (!fs.good()) {
        std::cerr << "Error: Could not read file " << filename << "\n";
        return images;
    }
    // Read all data lines
    for (std::string line; std::getline(fs, line);) {
        // Ignore comment lines
        if (line[0] == '#') {
            continue;
        }
        // Data line read, split on spaces
        const std::vector<std::string> columns = se::str_utils::split_str(line, ' ', true);
        // Ensure it has the expected number of columns
        if (columns.size() != 2) {
            std::cerr << "Error: Invalid format in data line " << images.size() + 1 << " of " << filename << "\n";
            std::cerr << "Error: The format of each line in a TUM depth.txt/rgb.txt file must be: "
                      << "timestamp filename\n";
            images.clear();
            return images;
        }
        // Ensure that the timestamp is a positive float
        if (!se::str_utils::is_float(columns[0], false)) {
            std::cerr << "Error: Invalid timestamp in data line " << images.size() + 1 << " of " << filename << "\n";
            std::cerr << "Error: The TUM timestamps must be non-negative floats\n";
            images.clear();
            return images;
        }
        // Add the image info
        images.emplace_back(line);
    }
    if (images.empty()) {
        std::cerr << "Error: Empty file " << filename << "\n";
    }
    return images;
}


/** Read a TUM groundtruth.txt into an std::vector of TUMPoseEntry.
  * Return an empty std::vector if the file was not in the correct format.
  */
std::vector<TUMPoseEntry> read_tum_ground_truth(const std::string& filename)
{
    std::vector<TUMPoseEntry> poses;
    std::ifstream fs(filename, std::ios::in);
    if (!fs.good()) {
        std::cerr << "Error: Could not read ground truth file " << filename << "\n";
        return poses;
    }
    // Read all data lines
    for (std::string line; std::getline(fs, line);) {
        // Ignore comment lines
        if (line[0] == '#') {
            continue;
        }
        // Data line read, split on spaces
        const std::vector<std::string> columns = se::str_utils::split_str(line, ' ', true);
        // Ensure it has the expected number of columns
        if (columns.size() != 8) {
            std::cerr << "Error: Invalid format in data line " << poses.size() + 1 << " of " << filename << "\n";
            std::cerr << "Error: The format of each line in a TUM ground truth file must be: "
                      << "timestamp tx ty tz qx qy qz qw\n";
            poses.clear();
            return poses;
        }
        // Ensure that the timestamp is a positive float
        if (!se::str_utils::is_float(columns[0], false)) {
            std::cerr << "Error: Invalid timestamp in data line " << poses.size() + 1 << " of " << filename << "\n";
            std::cerr << "Error: The TUM ground truth timestamps must be non-negative floats\n";
            poses.clear();
            return poses;
        }
        // Ensure that the rest of the data are floats
        for (size_t i = 0; i < columns.size(); ++i) {
            if (!se::str_utils::is_float(columns[i])) {
                std::cerr << "Error: Invalid pose in data line " << poses.size() + 1 << " of " << filename << "\n";
                std::cerr << "Error: The TUM ground truth pose data must be floats\n";
                poses.clear();
                return poses;
            }
        }
        // Add the pose
        poses.emplace_back(line);
    }
    if (poses.empty()) {
        std::cerr << "Error: Empty ground truth file " << filename << "\n";
    }
    return poses;
}


/** Search timestamps_to_search for the timestamp closest to query_timestamp.
  * Return the closest timestamp and its index in timestamps_to_search. If no
  * match is found return NaN and SIZE_MAX.
  */
std::pair<double, size_t> closest_timestamp(const double query_timestamp, const std::vector<double>& timestamps_to_search)
{
    double min_dist = INFINITY;
    double closest_timestamp = NAN;
    size_t idx = SIZE_MAX;
    for (size_t i = 0; i < timestamps_to_search.size(); ++i) {
        const double timestamp = timestamps_to_search[i];
        const double timestamp_dist = std::fabs(query_timestamp - timestamp);
        if (timestamp_dist < min_dist) {
            min_dist = timestamp_dist;
            closest_timestamp = timestamp;
            idx = i;
        }
    }
    return std::make_pair(closest_timestamp, idx);
}


/** Find the 2 poses that are right before and after the query_timestamp.
  * Return the previous and next pose. Return poses with NaN timestamps if no
  * match was found.
  */
std::pair<TUMPoseEntry, TUMPoseEntry> surrounding_poses(const double query_timestamp, const std::vector<TUMPoseEntry>& poses_to_search)
{
    TUMPoseEntry prev_pose;
    TUMPoseEntry next_pose;
    for (size_t i = 0; i < poses_to_search.size() - 1; ++i) {
        const TUMPoseEntry prev = poses_to_search[i];
        const TUMPoseEntry next = poses_to_search[i + 1];
        // TODO: need a better implementation to avoid unnecessary loops (JW)
        if (prev.timestamp == query_timestamp) {
            prev_pose = prev;
            next_pose = prev;
            break;
        }
        else if (next.timestamp == query_timestamp) {
            prev_pose = next;
            next_pose = next;
        }
        else if (prev.timestamp < query_timestamp && query_timestamp < next.timestamp) {
            prev_pose = prev;
            next_pose = next;
            break;
        }
    }
    return std::make_pair(prev_pose, next_pose);
}


/** Modify 2 vectors of depth and RGB filenames and timestamps so that their
  * elements correspond 1-to-1 with a maximum time difference of
  * max_timestamp_dist.
  */
void associate_images(std::vector<TUMImageEntry>& depth_images, std::vector<TUMImageEntry>& rgb_images, const double max_timestamp_dist)
{
    if (rgb_images.empty()) {
        return;
    }
    // Compute the timestamps of all depth images
    std::vector<double> depth_timestamps(depth_images.size());
    std::transform(depth_images.begin(), depth_images.end(), depth_timestamps.begin(), [](const auto& e) { return e.timestamp; });
    // Compute the timestamps of all RGB images
    std::vector<double> rgb_timestamps(rgb_images.size());
    std::transform(rgb_images.begin(), rgb_images.end(), rgb_timestamps.begin(), [](const auto& e) { return e.timestamp; });
    // Try to associate each depth image with an RGB image
    std::vector<TUMImageEntry> associated_depth_images;
    std::vector<TUMImageEntry> associated_rgb_images;
    std::set<size_t> associated_rgb_image_indices;
    for (size_t i = 0; i < depth_timestamps.size(); ++i) {
        const auto r = closest_timestamp(depth_timestamps[i], rgb_timestamps);
        const double timestamp_dist = std::fabs(depth_timestamps[i] - r.first);
        if (associated_rgb_image_indices.count(r.second)) {
            std::cerr << "Warning: skipping depth image " << depth_images[i].filename << " because the closest RGB image to it " << rgb_images[r.second].filename
                      << " has beed associated to another depth image\n";
        }
        else if (timestamp_dist <= max_timestamp_dist) {
            // Found a match, add to the associated images
            associated_depth_images.push_back(depth_images[i]);
            associated_rgb_images.push_back(rgb_images[r.second]);
            associated_rgb_image_indices.insert(r.second);
        }
        else {
            std::cerr << "Warning: Could not match depth image " << depth_images[i].filename << " to any RGB image.\nThe closest timestamp distance was " << timestamp_dist << " > "
                      << max_timestamp_dist << "\n";
        }
    }
    // Update the input vectors to contain only the associated images
    depth_images = associated_depth_images;
    rgb_images = associated_rgb_images;
}


/** Interpolate the gt_poses at the depth image timestamps.
  * The interpolation is only considered valid if the time difference of the
  * poses used is at most max_timestamp_dist. Depth and RGB image pairs with no
  * matching ground truth poses will be removed from depth_images and
  * rgb_images. Return an std::vector with the interpolated poses.
  */
std::vector<TUMPoseEntry>
interpolate_poses(std::vector<TUMImageEntry>& depth_images, std::vector<TUMImageEntry>& rgb_images, const std::vector<TUMPoseEntry>& gt_poses, const double max_timestamp_dist)
{
    std::vector<TUMImageEntry> output_depth_images;
    std::vector<TUMImageEntry> output_rgb_images;
    std::vector<TUMPoseEntry> associated_poses;
    // Interpolate the ground truth poses at the depth image timestamps
    for (size_t i = 0; i < depth_images.size(); ++i) {
        const double depth_timestamp = depth_images[i].timestamp;
        const auto poses = surrounding_poses(depth_timestamp, gt_poses);
        // Ignore depth and RGB images with no matching ground truth
        const double timestamp_diff = poses.second.timestamp - poses.first.timestamp;
        const double timestamp_dist = std::fabs(timestamp_diff);
        if (std::isnan(timestamp_dist) || timestamp_dist > max_timestamp_dist) {
            std::cerr << "Warning: Could not interpolate pose for depth image " << depth_images[i].filename << ".\nThe timestamp distance between the surrounding poses was " << timestamp_dist << " > "
                      << max_timestamp_dist << "\n";
            continue;
        }
        // Matched a ground truth pose, keep the image pair
        output_depth_images.push_back(depth_images[i]);
        if (!rgb_images.empty()) {
            output_rgb_images.push_back(rgb_images[i]);
        }
        // Compute the interpolation parameter in the interval [0, 1]. Just set it
        // to 0 if an exact match was found.
        const double t = (poses.second.timestamp != poses.first.timestamp) ? (depth_timestamp - poses.first.timestamp) / timestamp_diff : 0.0;
        // Interpolate the pose
        const Eigen::Vector3f p = (1.0f - t) * poses.first.position + t * poses.second.position;
        const Eigen::Quaternionf o = poses.first.orientation.slerp(t, poses.second.orientation);
        // Create a new TUMPoseEntry containing also the depth and RGB image filenames
        associated_poses.emplace_back(depth_timestamp, p, o, depth_images[i].filename, rgb_images.empty() ? "" : rgb_images[i].filename);
    }
    // Update the input vectors
    depth_images = output_depth_images;
    rgb_images = output_rgb_images;
    return associated_poses;
}


/** Generate a ground truth file from poses and write it in a temporary file.
  */
std::string write_ground_truth_tmp(const std::vector<TUMPoseEntry>& poses)
{
    // Open a temporary file
    const std::string tmp_filename = stdfs::temp_directory_path() / "tum_gt.txt";
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


// TUMReader implementation
constexpr float se::TUMReader::tum_inverse_scale_;
constexpr double se::TUMReader::max_match_timestamp_dist_;
constexpr double se::TUMReader::max_interp_timestamp_dist_;


se::TUMReader::TUMReader(const se::ReaderConfig& c) : se::Reader(c)
{
    inverse_scale_ = (c.inverse_scale != 0) ? c.inverse_scale : tum_inverse_scale_;

    // Ensure sequence_path_ refers to a valid TUM directory structure. Only depth data is required
    // to exist.
    if (!stdfs::is_directory(sequence_path_) || !stdfs::is_directory(sequence_path_ + "/depth") || !stdfs::is_regular_file(sequence_path_ + "/depth.txt")) {
        std::cerr << "Error: The TUM sequence path must be a directory that contains"
                  << " a depth subdirectory and a depth.txt file\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    // Read the image information from depth.txt.
    std::vector<TUMImageEntry> depth_images = read_tum_image_list(sequence_path_ + "/depth.txt");
    if (depth_images.empty()) {
        std::cerr << "Error: No TUM depth images found in depth.txt\n";
        status_ = se::ReaderStatus::error;
        return;
    }

    // Load colour image data if available.
    std::vector<TUMImageEntry> rgb_images;
    if (stdfs::is_directory(sequence_path_ + "/rgb") && stdfs::is_regular_file(sequence_path_ + "/rgb.txt")) {
        // Read the image information from rgb.txt.
        rgb_images = read_tum_image_list(sequence_path_ + "/rgb.txt");
        if (rgb_images.empty()) {
            std::cerr << "Error: No TUM colour images found in rgb.txt\n";
        }
        else {
            // Associate the depth with the RGB images.
            associate_images(depth_images, rgb_images, max_match_timestamp_dist_);
        }
    }
    else {
        std::cerr << "Warning: Colour images are unavailable, the TUM sequence doesn't"
                  << " contain an rgb subdirectory or an rgb.txt file\n";
    }

    // Read and associate the ground truth file if needed
    if (!ground_truth_file_.empty()) {
        // Read the the ground truth poses and timestamps
        const std::vector<TUMPoseEntry> gt_poses = read_tum_ground_truth(ground_truth_file_);
        if (gt_poses.empty()) {
            status_ = se::ReaderStatus::error;
            return;
        }
        // Interpolate the ground truth poses at the depth image timestamps
        const std::vector<TUMPoseEntry> associated_gt_poses = interpolate_poses(depth_images, rgb_images, gt_poses, max_interp_timestamp_dist_);
        if (associated_gt_poses.empty()) {
            std::cerr << "Error: Could not associate any ground truth poses to depth images\n";
            status_ = se::ReaderStatus::error;
            return;
        }
        // Generate the associated ground truth file
        const std::string generated_filename = write_ground_truth_tmp(associated_gt_poses);
        // Close the original ground truth file and open the generated one
        ground_truth_fs_.close();
        ground_truth_fs_.open(generated_filename, std::ios::in);
        if (!ground_truth_fs_.good()) {
            std::cerr << "Error: Could not read generated ground truth file " << generated_filename << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
    }

    // Get the filenames of the depth images.
    depth_filenames_.resize(depth_images.size());
    std::transform(depth_images.begin(), depth_images.end(), depth_filenames_.begin(), [](const auto& e) { return e.filename; });
    // Set the depth image resolution to that of the first depth image.
    const std::string first_depth_filename = sequence_path_ + "/" + depth_filenames_[0];
    cv::Mat image_data = cv::imread(first_depth_filename.c_str(), cv::IMREAD_UNCHANGED);
    if (image_data.empty()) {
        std::cerr << "Error: Could not read depth image " << first_depth_filename << "\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    depth_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);
    colour_image_res_ = depth_image_res_;
    class_id_image_res_ = colour_image_res_;

    // Get the filenames of the RGB images.
    rgb_filenames_.resize(rgb_images.size());
    std::transform(rgb_images.begin(), rgb_images.end(), rgb_filenames_.begin(), [](const auto& e) { return e.filename; });
    // Set the colour image resolution to that of the first colour image.
    const std::string first_rgb_filename = sequence_path_ + "/" + rgb_filenames_[0];
    image_data = cv::imread(first_rgb_filename.c_str(), cv::IMREAD_COLOR);
    if (image_data.empty()) {
        std::cerr << "Error: Could not read RGB image " << first_rgb_filename << "\n";
        rgb_filenames_.clear();
    }
    else {
        colour_image_res_ = Eigen::Vector2i(image_data.cols, image_data.rows);
    }

    num_frames_ = depth_filenames_.size();
    has_colour_ = !rgb_filenames_.empty();
}


void se::TUMReader::restart()
{
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
    }
}


std::string se::TUMReader::name() const
{
    return std::string("TUMReader");
}


se::ReaderStatus se::TUMReader::nextDepth(se::Image<float>& depth_image)
{
    if (frame_ >= num_frames_) {
        return se::ReaderStatus::error;
    }
    const std::string filename = sequence_path_ + "/" + depth_filenames_[frame_];

    // Read the image data.
    cv::Mat image_data = cv::imread(filename.c_str(), cv::IMREAD_UNCHANGED);
    cv::Mat depth_data;
    image_data.convertTo(depth_data, CV_32FC1, inverse_scale_);

    if (image_data.empty()) {
        return se::ReaderStatus::error;
    }

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


se::ReaderStatus se::TUMReader::nextColour(se::Image<rgb_t>& colour_image)
{
    if (frame_ >= num_frames_) {
        return se::ReaderStatus::error;
    }
    if (rgb_filenames_.empty()) {
        return se::Reader::nextColour(colour_image);
    }
    const std::string filename = sequence_path_ + "/" + rgb_filenames_[frame_];

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