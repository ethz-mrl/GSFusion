/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/io/mesh_io.hpp"

#include "se/common/str_utils.hpp"

namespace se {
namespace io {

bool has_supported_mesh_extension(const std::string& filename)
{
    for (const auto& extension : mesh_extensions) {
        if (se::str_utils::ends_with(filename, extension)) {
            return true;
        }
    }
    return false;
}

} // namespace io
} // namespace se
