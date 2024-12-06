/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/raycaster.hpp"


namespace se {
namespace raycaster {


void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals, const se::Image<Eigen::Vector3f>& point_cloud, const bool /* is_lhc */)
{
    const int width = point_cloud.width();
    const int height = point_cloud.height();

#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const Eigen::Vector3f point = point_cloud[x + width * y];
            if (point.z() == 0.f) {
                normals[x + y * width].x() = SE_INVALID;
                continue;
            }

            const Eigen::Vector2i p_left = Eigen::Vector2i(std::max(int(x) - 1, 0), y);
            const Eigen::Vector2i p_right = Eigen::Vector2i(std::min(x + 1, (int) width - 1), y);

            // Swapped to match the left-handed coordinate system of ICL-NUIM
            Eigen::Vector2i p_up, p_down;
            if (false) {
                p_up = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_down = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }
            else {
                p_down = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_up = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }

            const Eigen::Vector3f left = point_cloud[p_left.x() + width * p_left.y()];
            const Eigen::Vector3f right = point_cloud[p_right.x() + width * p_right.y()];
            const Eigen::Vector3f up = point_cloud[p_up.x() + width * p_up.y()];
            const Eigen::Vector3f down = point_cloud[p_down.x() + width * p_down.y()];

            if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0) {
                normals[x + y * width].x() = SE_INVALID;
                continue;
            }
            const Eigen::Vector3f dv_x = right - left;
            const Eigen::Vector3f dv_y = up - down;
            normals[x + y * width] = dv_x.cross(dv_y).normalized();
        } // x
    }     // y
}


void render_volume(uint32_t* volume_RGBA_image_data,
                   const Eigen::Vector2i& volume_RGBA_image_res,
                   const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                   const se::Image<Eigen::Vector3f>& surface_normals_M,
                   const Eigen::Vector3f& light_M,
                   const Eigen::Vector3f& ambient_M)
{
    render_volume_kernel(volume_RGBA_image_data, volume_RGBA_image_res, surface_point_cloud_M, surface_normals_M, light_M, ambient_M, [&](int /*pixel_idx*/, const Eigen::Vector3f& illumination) {
        const Eigen::Vector3i colour = (illumination * 255.0f).cast<int>();
        return se::pack_rgba(colour);
    });
}


void render_volume_scale(uint32_t* volume_RGBA_image_data,
                         const Eigen::Vector2i& volume_RGBA_image_res,
                         const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                         const se::Image<Eigen::Vector3f>& surface_normals_M,
                         const se::Image<int8_t>& surface_scale,
                         const Eigen::Vector3f& light_M,
                         const Eigen::Vector3f& ambient_M)
{
    render_volume_kernel(volume_RGBA_image_data, volume_RGBA_image_res, surface_point_cloud_M, surface_normals_M, light_M, ambient_M, [&](int pixel_idx, const Eigen::Vector3f& illumination) {
        const Eigen::Vector3i colour = illumination.cwiseProduct(se::colours::scale[surface_scale[pixel_idx]]).cast<int>();
        return se::pack_rgba(colour);
    });
}


void render_volume_colour(uint32_t* volume_RGBA_image_data,
                          const Eigen::Vector2i& volume_RGBA_image_res,
                          const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                          const se::Image<Eigen::Vector3f>& surface_normals_M,
                          const se::Image<rgb_t>& surface_colour,
                          const Eigen::Vector3f& light_M,
                          const Eigen::Vector3f& ambient_M)
{
    render_volume_kernel(volume_RGBA_image_data, volume_RGBA_image_res, surface_point_cloud_M, surface_normals_M, light_M, ambient_M, [&](int pixel_idx, const Eigen::Vector3f& illumination) {
        const rgb_t c = surface_colour[pixel_idx];
        const Eigen::Vector3i colour = illumination.cwiseProduct(Eigen::Vector3f(c.r, c.g, c.b)).cast<int>();
        return se::pack_rgba(colour);
    });
}


void render_volume_class_id(uint32_t* volume_RGBA_image_data,
                            const Eigen::Vector2i& volume_RGBA_image_res,
                            const se::Image<Eigen::Vector3f>& surface_point_cloud_M,
                            const se::Image<Eigen::Vector3f>& surface_normals_M,
                            const se::Image<semantics_t>& surface_class_id,
                            const Eigen::Vector3f& light_M,
                            const Eigen::Vector3f& ambient_M)
{
    render_volume_kernel(volume_RGBA_image_data, volume_RGBA_image_res, surface_point_cloud_M, surface_normals_M, light_M, ambient_M, [&](int pixel_idx, const Eigen::Vector3f& illumination) {
        const Eigen::Vector3i colour = illumination.cwiseProduct(se::colours::scale[surface_class_id[pixel_idx] % se::colours::scale.size()]).cast<int>();
        return se::pack_rgba(colour);
    });
}

} // namespace raycaster
} // namespace se
