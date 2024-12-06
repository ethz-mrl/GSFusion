/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_UTILS_HPP
#define SE_IMAGE_UTILS_HPP

#include <Eigen/Core>
#include <cstdint>
#include <string>


namespace se {
/**
 * Save a depth image with depth values in metres to a PNG.
 * The data is saved in a 16-bit image with the depth values scaled by scale.
 *
 * \param[in] depth_image_data Pointer to the float image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PNG file to create.
 * \param[in] scale            The number each float depth value is
 *                             multiplied with before being converted to a
 *                             uint16_t. By default the resulting PNG
 *                             contains depth values in millimetres.
 * \return                     0 on success, nonzero on error.
 */
int save_depth_png(const float* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename, const float scale = 1000.0f);


/**
 * Save a depth image with depth values in millimetres to a PNG.
 *
 * \param[in] depth_image_data Pointer to the 16-bit image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PNG file to create.
 * \return                     0 on success, nonzero on error.
 */
int save_depth_png(const uint16_t* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename);


/**
 * Load a PNG depth image into a buffer with depth values in metres.
 *
 * \param[in] depth_image_data Pointer to the loaded float image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PNG file to load.
 * \param[in] inverse_scale    The number each uint16_t depth value is
 *                             multiplied with in order to convert it to a
 *                             float value in metres. By default the
 *                             resulting PNG is assumed to contain depth
 *                             values in millimetres.
 * \return                     0 on success, nonzero on error.
 *
 * \warning The memory for the image buffer is allocated inside this
 * function. `delete[] *depth_image_data` must be called to free the memory.
 * width * height * sizeof(float) bytes are allocated.
 */
int load_depth_png(float** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename, const float inverse_scale = 1.0f / 1000.0f);


/**
 * Load a PNG depth image into a buffer with depth values in millimetres.
 *
 * \param[in] depth_image_data Pointer to the loaded 16-bit image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PNG file to load.
 * \return                     0 on success, nonzero on error.
 *
 * \warning The memory for the image buffer is allocated inside this
 * function. `delete[] *depth_image_data` must be called to free the memory.
 * width * height * sizeof(uint16_t) bytes are allocated.
 */
int load_depth_png(uint16_t** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename);


/**
 * Save a depth image with depth values in metres to a P2 PGM.
 * The data is saved in a 16-bit image with the depth values scaled by scale.
 *
 * \note For documentation on the structure of P2 PGM images see here
 * https://en.wikipedia.org/wiki/Netpbm_format
 *
 * \param[in] depth_image_data Pointer to the float image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PGM file to create.
 * \param[in] scale            The number each float depth value is
 *                             multiplied with before being converted to a
 *                             uint16_t. By default the resulting PNG
 *                             contains depth values in millimetres.
 * \return                     0 on success, nonzero on error.
 */
int save_depth_pgm(const float* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename, const float scale = 1000.0f);


/**
 * Save a depth image with depth values in millimetres to a P2 PGM.
 *
 * \note For documentation on the structure of P2 PGM images see here
 * https://en.wikipedia.org/wiki/Netpbm_format
 *
 * \param[in] depth_image_data Pointer to the 16-bit image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PGM file to create.
 * \return                     0 on success, nonzero on error.
 */
int save_depth_pgm(const uint16_t* depth_image_data, const Eigen::Vector2i& depth_image_res, const std::string& filename);


/**
 * Load a P2 PGM depth image into a buffer with depth values in metres.
 *
 * \param[in] depth_image_data Pointer to the loaded float image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PGM file to load.
 * \param[in] inverse_scale    The number each uint16_t depth value is
 *                             multiplied with in order to convert it to a
 *                             float value in metres. By default the
 *                             resulting PNG is assumed to contain depth
 *                             values in millimetres.
 * \return                     0 on success, nonzero on error.
 *
 * \warning The memory for the image buffer is allocated inside this
 * function. `delete[] *depth_image_data` must be called to free the memory.
 * width * height * sizeof(float) bytes are allocated.
 */
int load_depth_pgm(float** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename, const float inverse_scale = 1.0f / 1000.0f);


/**
 * Load a P2 PGM depth image into a buffer with depth values in millimeters.
 *
 * \param[in] depth_image_data Pointer to the loaded 16-bit image data.
 * \param[in] depth_image_res  Resolution of the depth image in pixels
 *                             (width and height).
 * \param[in] filename         The name of the PGM file to load.
 * \return                     0 on success, nonzero on error.
 *
 * \warning The memory for the image buffer is allocated inside this
 * function. `delete[] *depth_image_data` must be called to free the memory.
 * width * height * sizeof(uint16_t) bytes are allocated.
 */
int load_depth_pgm(uint16_t** depth_image_data, Eigen::Vector2i& depth_image_res, const std::string& filename);


static inline Eigen::Vector2i round_pixel(const Eigen::Vector2f& pixel_f);

} // namespace se

#include "impl/image_utils_impl.hpp"

#endif // SE_IMAGE_UTILS_HPP
