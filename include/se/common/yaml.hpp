/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_YAML_HPP
#define SE_YAML_HPP


#include <Eigen/Core>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>
#include <vector>


namespace se {
namespace yaml {


/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as a boolean and saves it
 *          in b. Shows a warning message on standard error and doesn't modify b if the YAML data is not
 *          a boolean or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] b            The bool value of the sub node
 */
void subnode_as_bool(const cv::FileNode& base_node, const std::string& subnode_name, bool& b);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as an int and saves it in i.
 *          Shows a warning message on standard error and doesn't modify i if the YAML data is not an
 *          int or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] i            The int value of the sub node
 */
void subnode_as_int(const cv::FileNode& base_node, const std::string& subnode_name, int& i);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as a float and saves it in f.
 *          Shows a warning message on standard error and doesn't modify f if the YAML data is not
 *          a float or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] f            The float value of the sub node
 */
void subnode_as_float(const cv::FileNode& base_node, const std::string& subnode_name, float& f);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as a string and saves it in s.
 *          Shows a warning message on standard error and doesn't modify s if the YAML data is not
 *          a string or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] s            The string value of the sub node
 */
void subnode_as_string(const cv::FileNode& base_node, const std::string& subnode_name, std::string& s);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as an std::vector and saves it in v.
 *          Shows a warning message on standard error and doesn't modify v if the YAML
 *          data is not a list or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] v            The vector value of the sub node
 */
template<typename T>
void subnode_as_vector(const cv::FileNode& base_node, const std::string& subnode_name, std::vector<T>& v);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as an Eigen::Vector3f and saves it in v.
 *          Shows a warning message on standard error and doesn't modify eigen_v3 if the YAML
 *          data is not a list or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] eigen_v3f    The Eigen::Vector3f value of the sub node
 */
void subnode_as_eigen_vector3f(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Vector3f& eigen_v3f);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as an Eigen::Vector3f and saves it in v.
 *          Shows a warning message on standard error and doesn't modify eigen_vx if the YAML
 *          data is not a list or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] eigen_v      The Eigen::Matrix<T, Eigen::Dynamic, 1> value of the sub node
 */
template<typename T>
void subnode_as_eigen_vector_x(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Matrix<T, Eigen::Dynamic, 1>& eigen_v);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as an Eigen::Vector3f and saves it in v.
 *          Shows a warning message on standard error and doesn't modify eigen_v3 if the YAML
 *          data is not a list or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] eigen_m3f    The Eigen::Matrix4f value of the sub node
 */
void subnode_as_eigen_matrix3f(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Matrix3f& eigen_m3f);

/**
 * \brief   Interprets the data in the subnode of base_node named subnode_name as an Eigen::Vector3f and saves it in v.
 *          Shows a warning message on standard error and doesn't modify eigen_v3 if the YAML
 *          data is not a list or doesn't exist.
 *
 * \param[in]  base_node    The reference to the base node
 * \param[in]  subnode_name The name of the sub node
 * \param[out] eigen_m4f    The Eigen::Matrix4f value of the sub node
 */
void subnode_as_eigen_matrix4f(const cv::FileNode& base_node, const std::string& subnode_name, Eigen::Matrix4f& eigen_m4f);


} // namespace yaml
} // namespace se


#include "impl/yaml_impl.hpp"


#endif // SE_YAML_HPP
