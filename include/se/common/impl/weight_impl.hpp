/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_WEIGHT_IMPL_HPP
#define SE_WEIGHT_IMPL_HPP

#include "se/common/math_util.hpp"

namespace se {
namespace weight {

static delta_weight_t div(delta_weight_t weight, delta_weight_t divisor)
{
    return math::div_ceil(weight, divisor);
}

static constexpr void increment(weight_t& weight, weight_t max_weight)
{
    // Use if instead of std::min to prevent overflow when max_weight is the maximum value of
    // weight_t.
    if (weight < max_weight) {
        weight++;
    }
}

} // namespace weight
} // namespace se

#endif // SE_WEIGHT_IMPL_HPP
