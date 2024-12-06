/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ANGLE_UTILS_HPP
#define SE_ANGLE_UTILS_HPP

#include <cmath>

namespace se {
namespace math {

constexpr inline double pi = 3.14159265358979323846;
/** τ = 2π
 */
constexpr inline double tau = 6.28318530717958647693;

/** Convert an angle expressed in radians to degrees.
 */
template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> degrees(T angle_rad);

/** Convert an angle expressed in degrees to radians.
 */
template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> radians(T angle_deg);

/** Wrap and angle expressed in radians to the interval [-π, π).
 */
template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> wrap_angle_pi(T angle_rad);

/** Wrap and angle expressed in radians to the interval [0, τ).
 */
template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> wrap_angle_tau(T angle_rad);

/** Compute the angle with the smallest magnitude in the interval [-π, π) to rotate from angle
 * start_rad to angle end_rad.
 */
template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> convex_angle(T start_rad, T end_rad);

} // namespace math
} // namespace se

#include "impl/angle_utils_impl.hpp"

#endif // SE_ANGLE_UTILS_HPP
