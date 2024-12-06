/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_UTILS_IMPL_HPP
#define SE_IMAGE_UTILS_IMPL_HPP

namespace se {
static inline Eigen::Vector2i round_pixel(const Eigen::Vector2f& pixel_f)
{
    return (pixel_f + Eigen::Vector2f::Constant(0.5f)).cast<int>();
}

} // namespace se

#endif // SE_IMAGE_UTILS_IMPL_HPP
