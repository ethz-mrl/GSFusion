/*
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Nils Funk
 * SPDX-FileCopyrightText: 2020-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Detect std::filesystem support, include the appropriate header and alias the
// namespace.

#ifndef __FILESYSTEM_HPP
#define __FILESYSTEM_HPP

#if __has_include(<filesystem>)
#    include <filesystem>
namespace stdfs = std::filesystem;

#elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
namespace stdfs = std::experimental::filesystem;

#else
// No std::filesystem support.
#    error A compiler with support for std::filesystem is required
#endif

#endif // __FILESYSTEM_HPP
