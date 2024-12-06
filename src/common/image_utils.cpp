/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/image_utils.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>


int se::save_depth_png(const float* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename, const float scale)
{
    // Scale the depth data and convert to uint16_t
    const size_t num_pixels = depth_image_res.prod();
    std::unique_ptr<uint16_t[]> depth_image_data_scaled(new uint16_t[num_pixels]);
#pragma omp parallel for
    for (size_t i = 0; i < num_pixels; ++i) {
        depth_image_data_scaled.get()[i] = std::roundf(scale * depth_image_data[i]);
    }
    // Save the uint16_t depth image
    return se::save_depth_png(depth_image_data_scaled.get(), depth_image_res, filename);
}


int se::save_depth_png(const uint16_t* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename)
{
    // Using a const_cast is safe since cv::imwrite does not modify its argument.
    cv::Mat depth_image(depth_image_res.y(), depth_image_res.x(), CV_16UC1, const_cast<uint16_t*>(depth_image_data));
    cv::imwrite(filename.c_str(), depth_image);

    return 0;
}


int se::load_depth_png(float** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename, const float inverse_scale)
{
    // Load the 16-bit depth data
    uint16_t* depth_image_data_scaled = nullptr;
    const int err = se::load_depth_png(&depth_image_data_scaled, depth_image_res, filename);
    // Return on error
    if (err) {
        delete[] depth_image_data_scaled;
        return err;
    }
    // Remove the scaling from the 16-bit depth data and convert to float
    const size_t num_pixels = depth_image_res.prod();
    *depth_image_data = new float[num_pixels];
#pragma omp parallel for
    for (size_t i = 0; i < num_pixels; ++i) {
        (*depth_image_data)[i] = inverse_scale * depth_image_data_scaled[i];
    }
    delete[] depth_image_data_scaled;
    return 0;
}


int se::load_depth_png(uint16_t** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename)
{
    // Load the image.
    cv::Mat depth_cv_image = cv::imread(filename.c_str(), cv::IMREAD_UNCHANGED);
    if (depth_cv_image.empty()) {
        *depth_image_data = nullptr;
        return 1;
    }

    depth_image_res = Eigen::Vector2i(depth_cv_image.cols, depth_cv_image.rows);
    *depth_image_data = new uint16_t[depth_image_res.x() * depth_image_res.y()];
    cv::Mat img(depth_cv_image.rows, depth_cv_image.cols, CV_16UC1, *depth_image_data);
    depth_cv_image.copyTo(img);

    return 0;
}


int se::save_depth_pgm(const float* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename, const float scale)
{
    // Scale the depth data and convert to uint16_t
    const size_t num_pixels = depth_image_res.prod();
    std::unique_ptr<uint16_t[]> depth_image_data_scaled(new uint16_t[num_pixels]);
#pragma omp parallel for
    for (size_t i = 0; i < num_pixels; ++i) {
        depth_image_data_scaled.get()[i] = std::roundf(scale * depth_image_data[i]);
    }
    // Save the uint16_t depth image
    return se::save_depth_pgm(depth_image_data_scaled.get(), depth_image_res, filename);
}


int se::save_depth_pgm(const uint16_t* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename)
{
    // Open the file for writing.
    std::ofstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Unable to write file " << filename << "\n";
        return 1;
    }

    // Write the PGM header.
    file << "P2\n";
    file << depth_image_res.x() << " " << depth_image_res.y() << "\n";
    file << UINT16_MAX << "\n";

    // Write the image data.
    for (int y = 0; y < depth_image_res.y(); y++) {
        for (int x = 0; x < depth_image_res.x(); x++) {
            const int pixel_idx = x + y * depth_image_res.x();
            file << depth_image_data[pixel_idx];
            // Do not add a whitespace after the last element of a row.
            if (x < depth_image_res.x() - 1) {
                file << " ";
            }
        }
        // Add a newline at the end of each row.
        file << "\n";
    }

    file.close();

    return 0;
}


int se::load_depth_pgm(float** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename, const float inverse_scale)
{
    // Load the 16-bit depth data
    uint16_t* depth_image_data_scaled = nullptr;
    const int err = se::load_depth_pgm(&depth_image_data_scaled, depth_image_res, filename);
    // Return on error
    if (err) {
        delete[] depth_image_data_scaled;
        return err;
    }
    // Remove the scaling from the 16-bit depth data and convert to float
    const size_t num_pixels = depth_image_res.prod();
    *depth_image_data = new float[num_pixels];
#pragma omp parallel for
    for (size_t i = 0; i < num_pixels; ++i) {
        (*depth_image_data)[i] = inverse_scale * depth_image_data_scaled[i];
    }
    delete[] depth_image_data_scaled;
    return 0;
}


int se::load_depth_pgm(uint16_t** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename)
{
    // Open the file for reading.
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Unable to read file " << filename << "\n";
        return 1;
    }

    // Read the file format.
    std::string pgm_format;
    std::getline(file, pgm_format);
    if (pgm_format != "P2") {
        std::cerr << "Invalid PGM format: " << pgm_format << "\n";
        return 1;
    }

    // Read the image size and allocate memory for the image.
    file >> depth_image_res.x() >> depth_image_res.y();
    const size_t num_pixels = depth_image_res.x() * depth_image_res.y();
    *depth_image_data = new uint16_t[num_pixels];

    // Read the maximum pixel value.
    size_t max_value;
    file >> max_value;
    if (max_value > UINT16_MAX) {
        std::cerr << "Invalid maximum depth value " << max_value << " > " << UINT16_MAX << "\n";
        return 1;
    }

    // Read the image data. Do not perform any scaling since in our cases the
    // pixel values represent distances.
    for (size_t pixel_idx = 0; pixel_idx < num_pixels; ++pixel_idx) {
        file >> (*depth_image_data)[pixel_idx];
    }

    file.close();

    return 0;
}
