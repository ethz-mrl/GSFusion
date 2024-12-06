/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_WEIGHT_HPP
#define SE_WEIGHT_HPP

namespace se {

/** \brief The type used for integration weights.
 */
typedef uint8_t weight_t;

/** \brief The type used for accumulating and propagating integration weights.
 */
typedef int16_t delta_weight_t;

namespace weight {

/** \brief Return the ceiling of weight divided by divisor.
 * This function works for both integer and floating point weights.
 */
static delta_weight_t div(delta_weight_t weight, delta_weight_t divisor);

/** \brief Increment weight by 1 while avoiding overflow.
 */
static constexpr void increment(weight_t& weight, weight_t max_weight);

} // namespace weight
} // namespace se

#include "impl/weight_impl.hpp"

#endif // SE_WEIGHT_HPP
