/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_YAML_IMPL_HPP
#define SE_YAML_IMPL_HPP


namespace se {
namespace yaml {


template<typename T>
void subnode_as_vector(const cv::FileNode& base_node, const std::string& subnode_name, std::vector<T>& v)
{
    const cv::FileNode subnode = base_node[subnode_name];
    if (subnode.isSeq() && subnode.size() >= 1) {
        v.clear();
        for (const auto& e : subnode) {
            v.push_back(static_cast<T>(e));
        }
    }
    else if (!subnode.empty()) {
        // Show warnings on invalid data
        std::cerr << "Warning: ";
        if (subnode.isSeq() && subnode.size() == 0) {
            std::cerr << "ignoring empty list in " << subnode_name;
        }
        else {
            std::cerr << "ignoring non-list data in " << subnode_name;
        }
        std::cerr << ", using default value {";
        for (size_t i = 0; i < v.size() - 1; i++) {
            std::cerr << v[i] << ", ";
        }
        std::cerr << v.back() << "}\n";
    }
}


template<typename T>
void subnode_as_eigen_vector_x(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Matrix<T, Eigen::Dynamic, 1>& eigen_v)
{
    std::vector<T> v;
    subnode_as_vector<T>(base_node, subnode_name, v);
    eigen_v = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(v.data(), v.size());
}


} // namespace yaml
} // namespace se

#endif // SE_YAML_IMPL_HPP
