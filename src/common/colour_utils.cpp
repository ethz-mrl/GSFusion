/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/colour_utils.hpp"


uint32_t gray_to_rgba(double h)
{
    constexpr double v = 0.75;
    double r = 0, g = 0, b = 0;
    if (v > 0) {
        constexpr double m = 0.25;
        constexpr double sv = 0.6667;
        h *= 6.0;
        const int sextant = static_cast<int>(h);
        const double fract = h - sextant;
        const double vsf = v * sv * fract;
        const double mid1 = m + vsf;
        const double mid2 = v - vsf;
        switch (sextant) {
        case 0:
            r = v;
            g = mid1;
            b = m;
            break;
        case 1:
            r = mid2;
            g = v;
            b = m;
            break;
        case 2:
            r = m;
            g = v;
            b = mid1;
            break;
        case 3:
            r = m;
            g = mid2;
            b = v;
            break;
        case 4:
            r = mid1;
            g = m;
            b = v;
            break;
        case 5:
            r = v;
            g = m;
            b = mid2;
            break;
        default:
            r = 0;
            g = 0;
            b = 0;
            break;
        }
    }
    return se::pack_rgba(r * 255, g * 255, b * 255, 255);
}


void se::depth_to_rgba(uint32_t* depth_RGBA_image_data, const float* depth_image_data, const Eigen::Vector2i& depth_image_res, const float min_depth, const float max_depth)
{
    const float inv_depth_range = 1.0f / (max_depth - min_depth);
#pragma omp parallel for
    for (int y = 0; y < depth_image_res.y(); y++) {
        const int row_offset = y * depth_image_res.x();
        for (int x = 0; x < depth_image_res.x(); x++) {
            const int pixel_idx = x + row_offset;
            const float depth = depth_image_data[pixel_idx];
            if (depth <= 0.0f || std::isnan(depth)) {
                depth_RGBA_image_data[pixel_idx] = 0xFF000000; // Black
            }
            else if (depth < min_depth) {
                depth_RGBA_image_data[pixel_idx] = 0xFF808080; // Gray
            }
            else if (depth > max_depth) {
                depth_RGBA_image_data[pixel_idx] = 0xFFFFFFFF; // White
            }
            else {
                const float normalized_depth = (depth - min_depth) * inv_depth_range;
                depth_RGBA_image_data[pixel_idx] = gray_to_rgba(normalized_depth);
            }
        }
    }
}


void se::semantics_to_rgba(uint32_t* class_id_RGBA_image_data, const semantics_t* class_id_image_data, const Eigen::Vector2i& class_id_image_res)
{
#pragma omp parallel for
    for (int i = 0; i < class_id_image_res.prod(); i++) {
        const semantics_t class_id = class_id_image_data[i];
        const Eigen::Vector3i colour = se::colours::scale[class_id % se::colours::scale.size()].cast<int>();
        class_id_RGBA_image_data[i] = se::pack_rgba(colour);
    }
}
