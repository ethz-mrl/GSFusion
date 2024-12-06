/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PINHOLE_CAMERA_IMPL_HPP
#define SE_PINHOLE_CAMERA_IMPL_HPP

namespace se {


inline float se::PinholeCamera::nearDistImpl(const Eigen::Vector3f& ray_S) const
{
    return near_plane / ray_S.normalized().z();
}


inline float se::PinholeCamera::farDistImpl(const Eigen::Vector3f& ray_S) const
{
    return far_plane / ray_S.normalized().z();
}


inline float se::PinholeCamera::measurementFromPointImpl(const Eigen::Vector3f& point_S) const
{
    return point_S.z();
}


inline std::string se::PinholeCamera::typeImpl()
{
    return "PinholeCamera";
}


} // namespace se

#endif // SE_PINHOLE_CAMERA_IMPL_HPP
