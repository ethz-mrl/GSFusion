/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/sensor/sensor.hpp"


void se::SensorBaseConfig::readYaml(const std::string& filename)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << filename << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << filename << "\n";
        return;
    }

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default sensor configuration, no \"sensor\" section found in " << filename << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_int(node, "width", width);
    se::yaml::subnode_as_int(node, "height", height);
    se::yaml::subnode_as_float(node, "near_plane", near_plane);
    se::yaml::subnode_as_float(node, "far_plane", far_plane);

    T_BS = Eigen::Matrix4f::Identity();

    if (!node["T_BS"].isNone()) {
        se::yaml::subnode_as_eigen_matrix4f(node, "T_BS", T_BS);
    }

    if (!node["t_BS"].isNone()) {
        Eigen::Vector3f t_BS;
        se::yaml::subnode_as_eigen_vector3f(node, "t_BS", t_BS);
        T_BS.topRightCorner<3, 1>() = t_BS;
    }

    if (!node["R_BS"].isNone()) {
        Eigen::Matrix3f R_BS;
        se::yaml::subnode_as_eigen_matrix3f(node, "R_BS", R_BS);
        T_BS.topLeftCorner<3, 3>() = R_BS;
    }
}
