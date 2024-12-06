/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/yaml.hpp"

#include "se/common/str_utils.hpp"


namespace se {
namespace yaml {


void subnode_as_bool(const cv::FileNode& base_node, const std::string& subnode_name, bool& b)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isString()) {
        // The OpenCV YAML reader does not support booleans, we have to parse them as strings.
        std::string s = static_cast<std::string>(subnode);
        se::str_utils::to_lower(s);
        if (s == "true") {
            b = true;
        }
        else if (s == "false") {
            b = false;
        }
        else {
            std::cerr << "Warning: ignoring non-bool string \"" << s << "\" in " << subnode_name << ", using default value \"" << b << "\"\n";
        }
    }
    else if (!subnode.empty()) {
        std::cerr << "Warning: ignoring non-bool data in " << subnode_name << ", using default value \"" << b << "\"\n";
    }
}


void subnode_as_int(const cv::FileNode& base_node, const std::string& subnode_name, int& i)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isInt()) {
        i = static_cast<int>(subnode);
    }
    else if (!subnode.empty()) {
        std::cerr << "Warning: ignoring non-int data in " << subnode_name << ", using default value \"" << i << "\"\n";
    }
}


void subnode_as_float(const cv::FileNode& base_node, const std::string& subnode_name, float& f)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isReal() || subnode.isInt()) {
        f = static_cast<float>(subnode);
    }
    else if (!subnode.empty()) {
        std::cerr << "Warning: ignoring non-float data in " << subnode_name << ", using default value \"" << f << "\"\n";
    }
}


void subnode_as_string(const cv::FileNode& base_node, const std::string& subnode_name, std::string& s)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isString()) {
        s = static_cast<std::string>(subnode);
    }
    else if (!subnode.empty()) {
        std::cerr << "Warning: ignoring non-string data in " << subnode_name << ", using default value \"" << s << "\"\n";
    }
}


void subnode_as_eigen_vector3f(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Vector3f& eigen_v3f)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isSeq() && subnode.size() == 3) {
        eigen_v3f = Eigen::Vector3f(subnode[0], subnode[1], subnode[2]);
    }
    else if (!subnode.empty()) {
        // Show warnings on invalid data
        std::cerr << "Warning: ";
        if (subnode.isSeq() && subnode.size() != 3) {
            std::cerr << "expected list of length 3 for " << subnode_name << " but got length " << subnode.size();
        }
        else {
            std::cerr << "ignoring non-list data in " << subnode_name;
        }
        std::cerr << ", using default value [" << eigen_v3f.x() << ", " << eigen_v3f.y() << ", " << eigen_v3f.z() << "]\n";
    }
}


void subnode_as_eigen_matrix3f(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Matrix3f& eigen_m3f)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isSeq() && subnode.size() == 9) {
        eigen_m3f << subnode[0], subnode[1], subnode[2], subnode[3], subnode[4], subnode[5], subnode[6], subnode[7], subnode[8];
    }
    else if (!subnode.empty()) {
        // Show warnings on invalid data
        std::cerr << "Warning: ";
        if (subnode.isSeq() && subnode.size() != 3) {
            std::cerr << "expected list of length 3 for " << subnode_name << " but got length " << subnode.size();
        }
        else {
            std::cerr << "ignoring non-list data in " << subnode_name;
        }
    }
}


void subnode_as_eigen_matrix4f(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Matrix4f& eigen_m4f)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isSeq() && subnode.size() == 16) {
        eigen_m4f << subnode[0], subnode[1], subnode[2], subnode[3], subnode[4], subnode[5], subnode[6], subnode[7], subnode[8], subnode[9], subnode[10], subnode[11], subnode[12], subnode[13],
            subnode[14], subnode[15];
    }
    else if (!subnode.empty()) {
        // Show warnings on invalid data
        std::cerr << "Warning: ";
        if (subnode.isSeq() && subnode.size() != 3) {
            std::cerr << "expected list of length 3 for " << subnode_name << " but got length " << subnode.size();
        }
        else {
            std::cerr << "ignoring non-list data in " << subnode_name;
        }
    }
}


} // namespace yaml
} // namespace se
