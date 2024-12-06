/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_TSDF_HPP
#define SE_TSDF_HPP

#include <cstdint>
#include <limits>
#include <type_traits>

namespace se {

/** \brief The type used for TSDF fields.
 * Valid types for tsdf_t are float and int16_t. int16_t will result in lower memory consumption but
 * can't be used in systems where sizeof(int) == sizeof(int16_t) as it will cause overflows. Other
 * types aren't used because int8_t results in quantization errors, int32_t, int64_t, double etc.
 * are needlessly large, unsigned integer types complicate scaling and certain tests.
 */
typedef int16_t tsdf_t;

/** \brief The type used for accumulating and propagating TSDF fields.
 * Valid types for delta_tsdf_t are float and int32_t since it must be be able to contain the sum of
 * 8 tsdf_t values.
 */
typedef std::conditional_t<std::is_same_v<tsdf_t, int16_t>, int32_t, float> delta_tsdf_t;

/** \brief The number values stored in tsdf_t must be divided by in order to receive the actual TSDF
 * value in the interval [-1, 1].
 */
constexpr tsdf_t tsdf_t_scale = std::is_integral_v<tsdf_t> ? std::numeric_limits<tsdf_t>::max() : 1;

static_assert(std::is_same_v<tsdf_t, float> || std::is_same_v<tsdf_t, int16_t>, "Only float and int16_t are supported for tsdf_t.");
static_assert(std::is_integral_v<tsdf_t> ? sizeof(int) >= 2 * sizeof(tsdf_t) : true, "Operations with tsdf_t won't cause overflows.");

} // namespace se

#endif // SE_TSDF_HPP
