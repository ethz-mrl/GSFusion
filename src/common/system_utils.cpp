/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/system_utils.hpp"

#include <fstream>
#include <string>

namespace se {
namespace system {

size_t memory_usage_self()
{
    size_t usage = 0;
#ifdef __linux__
    // Open the status file for the current process in the /proc pseudo-filesystem.
    std::ifstream file("/proc/self/status");
    if (!file.good()) {
        return usage;
    }
    for (std::string line; std::getline(file, line);) {
        // Find the line containing the virtual memory resident set size or the virtual memory swap
        // size.
        if (line.substr(0, 5) == "VmRSS" || line.substr(0, 6) == "VmSwap") {
            // Find the substring that contains the number.
            const auto num_start = line.find_first_of("0123456789");
            const auto num_len = line.find_last_of("0123456789") - num_start + 1;
            const std::string num = line.substr(num_start, num_len);
            // Find the substring that contains the unit.
            const auto unit_start = line.find_last_of(" ") + 1;
            const std::string unit = line.substr(unit_start);
            // Get the scale from the unit.
            size_t scale;
            switch (unit[0]) {
            case 'B': // B
                scale = 1;
                break;
            case 'k': // kB
                scale = 1024;
                break;
            case 'M': // MB
                scale = 1024 * 1024;
                break;
            case 'G': // GB
                scale = 1024 * 1024 * 1024;
                break;
            default: // Assume kB (seems to be the most common unit in /proc)
                scale = 1024;
                break;
            }
            usage += std::stoull(num) * scale;
        }
    }
#endif
    return usage;
}

} // namespace system
} // namespace se
