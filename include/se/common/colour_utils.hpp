/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COLOUR_UTILS_HPP
#define SE_COLOUR_UTILS_HPP

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "se/common/semantic_types.hpp"


namespace se {
namespace colours {
/**
 * The colours used for the various integration scales.
 */
static const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> scale =
    {{102, 194, 165}, {252, 141, 98}, {141, 160, 203}, {231, 138, 195}, {166, 216, 84}, {255, 217, 47}, {229, 196, 148}, {179, 179, 179}};
} // namespace colours

/**
 * Pack the individual RGBA channels into a single 32-bit unsigned integer.
 *
 * The uint32_t is stored in little-endian order in all common CPUs so the
 * alpha channel is stored in the MSB and the red channel in the LSB.
 * Red:   bits  0-7
 * Green: bits  8-15
 * Blue:  bits 16-23
 * Alpha: bits 24-31
 *
 * \param[in] r The value of the red channel.
 * \param[in] g The value of the green channel.
 * \param[in] b The value of the blue channel.
 * \param[in] a The value of the alpha channel.
 * \return The 32-bit unsigned integer RGBA value.
 */
static inline uint32_t pack_rgba(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a);


/**
 * Pack a color stored in an Eigen vector into a single 32-bit unsigned
 * integer. It is assumed that the R, G, B and A channels are stored in the
 * x, y, z and w members respectively. Their values are assumed to be in the
 * range [0, 1].
 *
 * \param[in] color The input color as an Eigen Vector.
 * \return The 32-bit unsigned integer RGBA value.
 */
static inline uint32_t pack_rgba(const Eigen::Vector4f& color);


/**
 * Pack a color stored in an Eigen vector into a single 32-bit unsigned
 * integer. It is assumed that the R, G and B channels are stored in the x, y
 * and z members respectively. Their values are assumed to be in the range
 * [0, 1]. The alpha channel is assumed to be completely opaque, e.g. 1.
 *
 * \param[in] color The input color as an Eigen Vector.
 * \return The 32-bit unsigned integer RGBA value.
 */
static inline uint32_t pack_rgba(const Eigen::Vector3f& color);


/**
 * Pack a color stored in an Eigen vector into a single 32-bit unsigned
 * integer. It is assumed that the R, G, B and A channels are stored in the
 * x, y, z and w members respectively. Their values are assumed to be in the
 * range [0x00, 0xFF].
 *
 * \param[in] color The input color as an Eigen Vector.
 * \return The 32-bit unsigned integer RGBA value.
 */
static inline uint32_t pack_rgba(const Eigen::Vector4i& color);


/**
 * Pack a color stored in an Eigen vector into a single 32-bit unsigned
 * integer. It is assumed that the R, G and B channels are stored in the x, y
 * and z members respectively. Their values are assumed to be in the range
 * [0x00, 0xFF]. The alpha channel is assumed to be completely opaque, e.g.
 * 0xFF.
 *
 * \param[in] color The input color as an Eigen Vector.
 * \return The 32-bit unsigned integer RGBA value.
 */
static inline uint32_t pack_rgba(const Eigen::Vector3i& color);


/**
 * Get the value of the red channel from a 32-bit packed RGBA value.
 *
 * \param[in] rgba The 32-bit packed RGBA value.
 * \return The value of the red channel.
 */
static inline uint8_t r_from_rgba(const uint32_t rgba);


/**
 * Get the value of the green channel from a 32-bit packed RGBA value.
 *
 * \param[in] rgba The 32-bit packed RGBA value.
 * \return The value of the green channel.
 */
static inline uint8_t g_from_rgba(const uint32_t rgba);


/**
 * Get the value of the blue channel from a 32-bit packed RGBA value.
 *
 * \param[in] rgba The 32-bit packed RGBA value.
 * \return The value of the blue channel.
 */
static inline uint8_t b_from_rgba(const uint32_t rgba);


/**
 * Get the value of the alpha channel from a 32-bit packed RGBA value.
 *
 * \param[in] rgba The 32-bit packed RGBA value.
 * \return The value of the alpha channel.
 */
static inline uint8_t a_from_rgba(const uint32_t rgba);


/**
 * Blend two RGBA colors based on the value of the blending parameter alpha.
 * Returns a color alpha * rgba_1 + (1 - alpha) * rgba_2. The values of
 * alpha are assumed to be in the range [0, 1].
 *
 * \note Code from https://stackoverflow.com/a/12016968
 *
 * \param[in] rgba_1 A 32-bit packed RGBA value.
 * \param[in] rgba_2 A 32-bit packed RGBA value.
 * \param[in] alpha The value of the blending parameter.
 * \return The 32-bit RGBA value of the blended color.
 *
 * \warning Swapping the rgba_1 with rgba_2 while keeping the same value for
 * alpha will not always produce the same result.
 */
static inline uint32_t blend(const uint32_t rgba_1, const uint32_t rgba_2, const float alpha);


static inline void rgb_to_rgba(const uint8_t* rgb, uint32_t* rgba, size_t num_pixels);


static inline void rgba_to_rgb(const uint32_t* rgba, uint8_t* rgb, size_t num_pixels);


/**
 * Convert a depth image to an RGBA image to allow visualizing it.
 * The depth image is scaled using the minimum and maximum depth values to
 * increase contrast.
 *
 * \param[in] depth_RGBA_image_data Pointer to the ouput RGBA image data.
 * \param[in] depth_image_data      Pointer to the input depth image data.
 * \param[in] depth_image_res       Resolution of the depth image in pixels
 *                                  (width and height).
 * \param[in] min_depth             The minimum possible depth value.
 * \param[in] max_depth             The maximum possible depth value.
 */
void depth_to_rgba(uint32_t* depth_RGBA_image_data, const float* depth_image_data, const Eigen::Vector2i& depth_image_res, const float min_depth, const float max_depth);

/**
 * Convert a class ID image to an RGBA image to allow visualizing it.
 * The image is colourized using se::colours::scale.
 *
 * \param[in] class_id_RGBA_image_data Pointer to the ouput RGBA image data.
 * \param[in] class_id_image_data      Pointer to the input class ID image data.
 * \param[in] class_id_image_res       Resolution of the input and output images in pixels (width
 *                                     and height).
 */
void semantics_to_rgba(uint32_t* class_id_RGBA_image_data, const semantics_t* class_id_image_data, const Eigen::Vector2i& class_id_image_res);
} // namespace se

#include "impl/colour_utils_impl.hpp"

#endif // SE_COLOUR_UTILS_HPP
