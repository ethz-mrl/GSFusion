/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_COLOUR_UTILS_IMPL_HPP
#define SE_COLOUR_UTILS_IMPL_HPP

namespace se {
static inline uint32_t pack_rgba(const uint8_t r, const uint8_t g, const uint8_t b, const uint8_t a)
{
    return (a << 24) + (b << 16) + (g << 8) + r;
}


static inline uint32_t pack_rgba(const Eigen::Vector4f& color)
{
    return (static_cast<uint8_t>(color.w() * 255) << 24) + (static_cast<uint8_t>(color.z() * 255) << 16) + (static_cast<uint8_t>(color.y() * 255) << 8) + static_cast<uint8_t>(color.x() * 255);
}


static inline uint32_t pack_rgba(const Eigen::Vector3f& color)
{
    return (0xFF << 24) + (static_cast<uint8_t>(color.z() * 255) << 16) + (static_cast<uint8_t>(color.y() * 255) << 8) + static_cast<uint8_t>(color.x() * 255);
}


static inline uint32_t pack_rgba(const Eigen::Vector4i& color)
{
    return (color.w() << 24) + (color.z() << 16) + (color.y() << 8) + color.x();
}


static inline uint32_t pack_rgba(const Eigen::Vector3i& color)
{
    return (0xFF << 24) + (color.z() << 16) + (color.y() << 8) + color.x();
}


static inline uint8_t r_from_rgba(const uint32_t rgba)
{
    return (uint8_t) rgba;
}


static inline uint8_t g_from_rgba(const uint32_t rgba)
{
    return (uint8_t)(rgba >> 8);
}


static inline uint8_t b_from_rgba(const uint32_t rgba)
{
    return (uint8_t)(rgba >> 16);
}


static inline uint8_t a_from_rgba(const uint32_t rgba)
{
    return (uint8_t)(rgba >> 24);
}


static inline uint32_t blend(const uint32_t rgba_1, const uint32_t rgba_2, const float alpha)
{
    const uint8_t r = static_cast<uint8_t>(round(alpha * r_from_rgba(rgba_1) + (1 - alpha) * r_from_rgba(rgba_2)));
    const uint8_t g = static_cast<uint8_t>(round(alpha * g_from_rgba(rgba_1) + (1 - alpha) * g_from_rgba(rgba_2)));
    const uint8_t b = static_cast<uint8_t>(round(alpha * b_from_rgba(rgba_1) + (1 - alpha) * b_from_rgba(rgba_2)));
    const uint8_t a = static_cast<uint8_t>(round(alpha * a_from_rgba(rgba_1) + (1 - alpha) * a_from_rgba(rgba_2)));

    return pack_rgba(r, g, b, a);
}


static inline void rgb_to_rgba(const uint8_t* rgb, uint32_t* rgba, size_t num_pixels)
{
#pragma omp parallel for
    for (size_t p = 0; p < num_pixels; ++p) {
        const uint8_t r = rgb[3 * p + 0];
        const uint8_t g = rgb[3 * p + 1];
        const uint8_t b = rgb[3 * p + 2];
        rgba[p] = pack_rgba(r, g, b, 0xFF);
    }
}


static inline void rgba_to_rgb(const uint32_t* rgba, uint8_t* rgb, size_t num_pixels)
{
#pragma omp parallel for
    for (size_t p = 0; p < num_pixels; ++p) {
        rgb[3 * p + 0] = r_from_rgba(rgba[p]);
        rgb[3 * p + 1] = g_from_rgba(rgba[p]);
        rgb[3 * p + 2] = b_from_rgba(rgba[p]);
    }
}
} // namespace se

#endif // SE_COLOUR_UTILS_IMPL_HPP
