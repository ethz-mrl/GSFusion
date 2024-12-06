/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_PREPROCESSOR_IMPL_HPP
#define SE_PREPROCESSOR_IMPL_HPP

namespace se {
namespace preprocessor {


template<typename SensorT>
void depth_to_point_cloud(se::Image<Eigen::Vector3f>& point_cloud_C, const se::Image<float>& depth_image, const SensorT& sensor)
{
#pragma omp parallel for
    for (int y = 0; y < depth_image.height(); y++) {
        for (int x = 0; x < depth_image.width(); x++) {
            const Eigen::Vector2i pixel(x, y);
            if (depth_image(pixel.x(), pixel.y()) > 0) {
                const Eigen::Vector2f pixel_f = pixel.cast<float>();
                Eigen::Vector3f ray_dir_C;
                sensor.model.backProject(pixel_f, &ray_dir_C);
                point_cloud_C[pixel.x() + pixel.y() * depth_image.width()] = depth_image(pixel.x(), pixel.y()) * ray_dir_C;
            }
            else {
                point_cloud_C[pixel.x() + pixel.y() * depth_image.width()] = Eigen::Vector3f::Zero();
            }
        }
    }
}


} // namespace preprocessor
} // namespace se

#endif // SE_PREPROCESSOR_IMPL_HPP
