/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_CONFIDENCE_HPP
#define SE_CONFIDENCE_HPP

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>

#include "type_util.hpp"

namespace se {

/** \brief Store the semantic class confidence for N classes.
 * Each confidence value is scaled from the interval [0, 1] to the interval [0, 255] inclusive.
 */
template<size_t N, typename T = uint8_t>
struct Confidence : std::array<T, N> {
    public:
    /** \brief The ID of the class with the highest confidence. */
    semantics_t class_id = 0;

    /** \brief Update the array with a measurement of class ID class_id. */
    template<typename U>
    constexpr void merge(semantics_t class_id, U weight);

    template<typename U>
    constexpr Confidence<N, U> cast() const;

    template<typename U,
             std::enable_if_t<(std::is_integral_v<T> && std::is_integral_v<U>) || (std::is_floating_point_v<T> && std::is_floating_point_v<U>), bool> = true,
             std::enable_if_t<sizeof(T) >= sizeof(U), bool> = true>
    constexpr Confidence& operator+=(const Confidence<N, U>& rhs);

    friend constexpr Confidence operator+(Confidence lhs, const Confidence& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    template<typename U,
             std::enable_if_t<(std::is_integral_v<T> && std::is_integral_v<U>) || (std::is_floating_point_v<T> && std::is_floating_point_v<U>), bool> = true,
             std::enable_if_t<sizeof(T) >= sizeof(U), bool> = true>
    constexpr Confidence& operator-=(const Confidence<N, U>& rhs);

    friend constexpr Confidence operator-(Confidence lhs, const Confidence& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    constexpr Confidence& operator*=(U rhs);

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    friend constexpr Confidence operator*(Confidence lhs, U rhs)
    {
        lhs *= rhs;
        return lhs;
    }

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    constexpr Confidence& operator/=(U rhs);

    template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool> = true>
    friend constexpr Confidence operator/(Confidence lhs, U rhs)
    {
        lhs /= rhs;
        return lhs;
    }

    private:
    template<typename U>
    constexpr void update_0(semantics_t class_id, U weight);
    template<typename U>
    constexpr void update_1(semantics_t class_id, U weight);
};

template<size_t N>
using Confidence16s = Confidence<N, int16_t>;

} // namespace se

#include "impl/confidence_impl.hpp"

#endif // SE_CONFIDENCE_HPP
