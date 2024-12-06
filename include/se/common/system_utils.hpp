/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SYSTEM_UTILS_HPP
#define SE_SYSTEM_UTILS_HPP

#include <cstdlib>

namespace se {
namespace system {

/** Return the memory usage in bytes of the calling process.
 *
 * This function depends on the underlying OS. It's implemented for the following:
 * - Linux: It parses /proc/self/status for the memory usage (VmRSS + VmSwap). The returned value
 *   will be an upper bound on the memory the program actually needs due to how memory allocations
 *   are handled by the OS.
 * - Other: Always returns 0.
 *
 * \return The memory usage in bytes.
 */
size_t memory_usage_self();

} // namespace system
} // namespace se

#endif // SE_SYSTEM_UTILS_HPP
