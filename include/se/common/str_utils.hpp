/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_STR_UTILS_HPP
#define SE_STR_UTILS_HPP


#include <Eigen/Core>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>


namespace se {
namespace str_utils {


/**
 * \brief Verify if a string starts with a given prefix.
 *
 * \param[in] s         The full string
 * \param[in] prefix    The prefix to be evaluated
 *
 * \return True if the string starts with the given prefix, false otherwise
 */
bool begins_with(const std::string& s, const std::string& prefix);

/**
 * \brief Verify if a string starts with a given prefix.
 *
 * \param[in] s         The full string
 * \param[in] suffix    The prefix to be evaluated
 *
 * \return True if the string ends with the given suffix, false otherwise
 */
bool ends_with(const std::string& s, const std::string& suffix);

/**
 * \brief Verify if a string is a integer.
 *
 * \param[in] s                 The string to be evaluated
 * \param[in] accept_negative   Accept negative integers
 *
 * \return True if the string is an integer, false otherwise
 */
bool is_int(const std::string& s, const bool accept_negative = true);

/**
 * \brief Verify if a string is a float.
 *
 * \param[in] s                 The string to be evaluated
 * \param[in] accept_negative   Accept negative floats
 *
 * \return True if the string is an float, false otherwise
 */
bool is_float(const std::string& s, const bool accept_negative = true);

/**
 * \brief Remove a given prefix from a string.
 *
 * \param[in,out] s         The full string
 * \param[in]     prefix    The prefix to be removed from the string
 */
void remove_prefix(std::string& s, const std::string& prefix);

/**
 * \brief Remove a given suffix from a string.
 *
 * \param[in,out] s         The full string
 * \param[in]     suffix    The suffix to be removed from the string
 */
void remove_suffix(std::string& s, const std::string& suffix);

/**
 * \brief Split a string into a vector of substrings based on a delimiter.
 *
 * \param[in] s             The string to split.
 * \param[in] delim         The delimiter to use.
 * \param[in] ignore_consec Treat consecutive delimiters as a single delimiter.
 *
 * \return The vector containing the substrings.
 */
std::vector<std::string> split_str(const std::string& s, const char delim, const bool ignore_consec = false);

/**
 * \brief Convert a string to all lower case characters.
 *
 * \param[in,out] s     The string to be converted
 */
void to_lower(std::string& s);

/**
 * \brief Convert a string to all upper case characters.
 *
 * \param[in,out] s     The string to be converted
 */
void to_upper(std::string& s);

static constexpr int default_width = 33;

/**
 * \brief Convert a matrix name and Eigen::Matrix value to a standardised string output.
 *
 * \tparam MatrixT
 * \param[in] M         The Eigen::Matrix
 * \param[in] M_name    The displayed name of the matrix
 * \param[in] width     The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
template<typename EigenMatrixT>
std::string eigen_matrix_to_pretty_str(const EigenMatrixT& M, const std::string& M_name = "", const int width = default_width);

/**
 * \brief Convert a matrix name and Eigen::Vector value to a standardised string output.
 *
 * \tparam VectorT
 * \param[in] v         The Eigen::Vector
 * \param[in] v_name    The displayed name of the vector
 * \param[in] e_name    The displayed name of vector elements
 * \param[in] width     The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
template<typename EigenVectorT>
std::string eigen_vector_to_pretty_str(const EigenVectorT& v, const std::string& v_name = "", const std::vector<std::string>& e_names = {}, const int width = default_width);

/**
 * \brief Convert a matrix name and Eigen::Vector value to a standardised string output.
 *
 * \tparam T vector type
 * \param[in] v         The standard vector
 * \param[in] v_name    The displayed name of the vector
 * \param[in] width     The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
template<typename T>
std::string vector_to_pretty_str(const std::vector<T>& v, const std::string& v_name = "", const int width = default_width);

/**
 * \brief Convert a volume name and value to a standardised string output (values seperated by 'x').
 *
 * \tparam MatrixT
 * \param[in] vol         The volume
 * \param[in] vol_name    The displayed name of the volume
 * \param[in] vol_unit    The displayed unit of the volume
 * \param[in] width       The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
template<typename EigenVector3T>
std::string volume_to_pretty_str(const EigenVector3T& vol, const std::string& vol_name = "", const int width = default_width);

/**
 * \brief Convert a bool name and value to a standardised string output.
 *
 * \param[in] state         The bool
 * \param[in] state_name    The displayed name of the bool
 * \param[in] width         The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
std::string bool_to_pretty_str(const bool state, const std::string& state_name = "", const int width = default_width);

/**
 * \brief Convert a string name and value to a standardised string output.
 *
 * \param[in] string         The string
 * \param[in] string_name    The displayed name of the string
 * \param[in] width          The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
std::string str_to_pretty_str(const std::string& string, const std::string& string_name = "", const int width = default_width);

/**
 * \brief Convert a matrix name, value and unit to a standardised string output.
 *
 * \tparam ValueT
 * \param[in] val         The value
 * \param[in] val_name    The displayed name of the value
 * \param[in] val_unit    The displayed unit of the value
 * \param[in] width       The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
template<typename ValueT>
std::string value_to_pretty_str(const ValueT& val, const std::string& val_name = "", const std::string& val_unit = "", const int width = default_width);

/**
 * \brief Convert header name to a standardised string output.
 *
 * \param[in] header_name         The header
 * \param[in] width               The starting position of the value in the string (default = default_width)
 *
 * \return The pretty string
 */
std::string header_to_pretty_str(const std::string& header_name, const int width = default_width);

/** Return the result of expanding a leading ~ in path.
 * \note Also expands environment variables since it uses wordexp(3) internally.
 * \note Currently only implemented for POSIX systems.
 */
std::string expand_user(const std::string& path);

/** Return the path resulting by resolving relative_path relative to the directory base_dir. If
 * relative_path is an absolute path it is returned as-is.
 */
std::string resolve_relative_path(const std::string& relative_path, const std::string& base_dir);

} // namespace str_utils
} // namespace se


#include "impl/str_utils_impl.hpp"


#endif // SE_STR_UTILS_HPP
