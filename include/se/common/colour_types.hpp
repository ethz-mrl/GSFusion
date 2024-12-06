/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COLOUR_TYPES_HPP
#define SE_COLOUR_TYPES_HPP

#include <cstdint>
#include <type_traits>

namespace se {

/** \brief A type used to store an RGB colour.
 * It allows performing common mathematical operations and accumulating colours of a narrower type
 * into colours of a wider type.
 */
template<typename T, std::enable_if_t<std::is_arithmetic_v<T>, bool> = true>
struct rgb {
    T r;
    T g;
    T b;

    friend constexpr bool operator==(const rgb& lhs, const rgb& rhs)
    {
        return (lhs.r == rhs.r) && (lhs.g == rhs.g) && (lhs.b == rhs.b);
    }

    friend constexpr bool operator!=(const rgb& lhs, const rgb& rhs)
    {
        return !(lhs == rhs);
    }

    template<typename U,
             std::enable_if_t<(std::is_integral_v<T> && std::is_integral_v<U>) || (std::is_floating_point_v<T> && std::is_floating_point_v<U>), bool> = true,
             std::enable_if_t<sizeof(T) >= sizeof(U), bool> = true>
    constexpr rgb& operator+=(const rgb<U>& rhs)
    {
        r += rhs.r;
        g += rhs.g;
        b += rhs.b;
        return *this;
    }

    friend constexpr rgb operator+(rgb lhs, const rgb& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    template<typename U,
             std::enable_if_t<(std::is_integral_v<T> && std::is_integral_v<U>) || (std::is_floating_point_v<T> && std::is_floating_point_v<U>), bool> = true,
             std::enable_if_t<sizeof(T) >= sizeof(U), bool> = true>
    constexpr rgb& operator-=(const rgb<U>& rhs)
    {
        r -= rhs.r;
        g -= rhs.g;
        b -= rhs.b;
        return *this;
    }

    friend constexpr rgb operator-(rgb lhs, const rgb& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    constexpr rgb& operator*=(U rhs)
    {
        r = (r * rhs) + U(0.5f);
        g = (g * rhs) + U(0.5f);
        b = (b * rhs) + U(0.5f);
        return *this;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    friend constexpr rgb operator*(rgb lhs, U rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    friend constexpr rgb operator*(U lhs, rgb rhs)
    {
        rhs *= lhs;
        return rhs;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    constexpr rgb& operator/=(U rhs)
    {
        r /= rhs;
        g /= rhs;
        b /= rhs;
        return *this;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    friend constexpr rgb operator/(rgb lhs, U rhs)
    {
        lhs /= rhs;
        return lhs;
    }

    template<typename U>
    constexpr rgb<U> cast() const
    {
        return rgb<U>{static_cast<U>(r), static_cast<U>(g), static_cast<U>(b)};
    }
};

typedef rgb<uint8_t> rgb_t;
typedef rgb<int16_t> rgb16s_t;

} // namespace se

#endif // SE_COLOUR_TYPES_HPP
