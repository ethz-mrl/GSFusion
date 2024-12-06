/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ANGLE_UTILS_IMPL_HPP
#define SE_ANGLE_UTILS_IMPL_HPP

namespace se {
namespace math {

template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> degrees(T angle_rad)
{
    return angle_rad / static_cast<T>(pi) * static_cast<T>(180);
}

template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> radians(T angle_deg)
{
    return angle_deg / static_cast<T>(180) * static_cast<T>(pi);
}

template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> wrap_angle_pi(T angle_rad)
{
    return wrap_angle_tau(angle_rad + static_cast<T>(pi)) - static_cast<T>(pi);
}

template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> wrap_angle_tau(T angle_rad)
{
    T angle = std::fmod(angle_rad, static_cast<T>(tau));
    if (angle < 0) {
        angle += static_cast<T>(tau);
    }
    return angle;
}

template<typename T>
constexpr inline std::enable_if_t<std::is_floating_point_v<T>, T> convex_angle(T start_rad, T end_rad)
{
    // Compute the angle difference in the interval (-2π, 2π).
    float angle_diff = wrap_angle_tau(end_rad) - wrap_angle_tau(start_rad);
    // Select a smaller angle if needed.
    if (angle_diff >= static_cast<T>(pi)) {
        angle_diff -= static_cast<T>(tau);
    }
    else if (angle_diff < -static_cast<T>(pi)) {
        angle_diff += static_cast<T>(tau);
    }
    return wrap_angle_pi(angle_diff);
}

} // namespace math
} // namespace se


#endif // SE_ANGLE_UTILS_IMPL_HPP
