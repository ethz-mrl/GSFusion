/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DENSE_POOLING_IMAGE_HPP
#define SE_DENSE_POOLING_IMAGE_HPP

namespace se {


template<typename SensorImplType>
DensePoolingImage<SensorImplType>::DensePoolingImage(const se::Image<float>& /* depth_image */)
{
    std::cout << "DensePoolingImage not implemented" << std::endl;
}


template<typename SensorImplType>
Pixel DensePoolingImage<SensorImplType>::conservativeQuery(const Eigen::Vector2i& /* bb_min */, const Eigen::Vector2i& /* bb_max */) const
{
    return Pixel::unknownPixel();
}


template<typename SensorImplType>
Pixel DensePoolingImage<SensorImplType>::poolBoundingBox(int /* u_min */, int /* u_max */, int /* v_min */, int /* v_max */) const
{
    return Pixel::unknownPixel();
}


template<typename SensorImplType>
bool DensePoolingImage<SensorImplType>::inImage(const int u, const int v) const
{
    if (u >= 0 && u < static_cast<Value>(image_width_) && v >= 0 && v < static_cast<Value>(image_height_)) {
        return true;
    }
    return false;
}


/// Pinhole Camera implementation
template<>
inline DensePoolingImage<PinholeCamera>::DensePoolingImage(const se::Image<float>& depth_map) : image_width_(depth_map.width()), image_height_(depth_map.height())
{
    TICK("image-construction")
    const int image_max_dim = std::min(image_width_, image_height_);
    image_max_level_ = static_cast<int>(log2((image_max_dim - 1) / 2) + 2) - 1;

    for (int l = 0; l <= image_max_level_; l++)
        pooling_image_.emplace_back(image_width_ * image_height_);

        // Initalize image frame at single pixel resolution
#pragma omp parallel for
    for (int v = 0; v < image_height_; v++) {
        for (int u = 0; u < image_width_; u++) {
            Value pixel_depth = (depth_map.data())[u + v * image_width_];
            if (pixel_depth <= 0) {
                pooling_image_[0][u + v * image_width_] = Pixel::unknownPixel(); // state_1 := inside (0); state_2 := unknown (2)
            }
            else {
                Pixel& pixel = pooling_image_[0][u + v * image_width_];
                pixel = Pixel::knownPixel(); // state_1 := inside (0); state_2 := known (0)
                pixel.min = pixel_depth;
                pixel.max = pixel_depth;
            }
        }
    }

    size_t num_pixel = image_width_ * image_height_;
    Img default_image = std::vector<Pixel>(num_pixel, Pixel::crossingKnownPixel()); // state_1 := crossing (1); state_2 := known (0)

    pooling_image_[1] = default_image;

    // Initalize first pixel batch at 3x3 batch resolution
#pragma omp parallel for
    for (int y = 0; y < image_height_; y++) {
        for (int x = 0; x < image_width_; x++) {
            Pixel& pixel = pooling_image_[1][x + image_width_ * y];

            if (y >= 1 && y < image_height_ - 1 && x >= 1 && x < image_width_ - 1) {
                pixel.status_crossing = Pixel::statusCrossing::inside; // inside (0)
            }


            int top = (y > 0) ? y - 1 : 0;
            int bottom = (y < image_height_ - 1 ? y + 1 : image_height_ - 1);
            int left = (x > 0) ? x - 1 : 0;
            int right = (x < image_width_ - 1 ? x + 1 : image_width_ - 1);

            pixel.min = std::min(pooling_image_[0][x + y * image_width_].min,
                                 std::min(std::min(std::min(pooling_image_[0][left + top * image_width_].min, pooling_image_[0][x + top * image_width_].min),
                                                   std::min(pooling_image_[0][right + top * image_width_].min, pooling_image_[0][right + y * image_width_].min)),
                                          std::min(std::min(pooling_image_[0][right + bottom * image_width_].min, pooling_image_[0][x + bottom * image_width_].min),
                                                   std::min(pooling_image_[0][left + bottom * image_width_].min, pooling_image_[0][left + y * image_width_].min))));
            pixel.max = std::max(pooling_image_[0][x + y * image_width_].max,
                                 std::max(std::max(std::max(pooling_image_[0][left + top * image_width_].max, pooling_image_[0][x + top * image_width_].max),
                                                   std::max(pooling_image_[0][right + top * image_width_].max, pooling_image_[0][right + y * image_width_].max)),
                                          std::max(std::max(pooling_image_[0][right + bottom * image_width_].max, pooling_image_[0][x + bottom * image_width_].max),
                                                   std::max(pooling_image_[0][left + bottom * image_width_].max, pooling_image_[0][left + y * image_width_].max))));
            int unknown_factor = pooling_image_[0][x + y * image_width_].status_known + pooling_image_[0][left + top * image_width_].status_known
                + pooling_image_[0][x + top * image_width_].status_known + pooling_image_[0][right + top * image_width_].status_known + pooling_image_[0][right + y * image_width_].status_known
                + pooling_image_[0][right + bottom * image_width_].status_known + pooling_image_[0][x + bottom * image_width_].status_known
                + pooling_image_[0][left + bottom * image_width_].status_known + pooling_image_[0][left + y * image_width_].status_known;
            if (unknown_factor == 18)                                // All pixel are unknown -> 9 * unknown (2) = 18
                pixel.status_known = Pixel::statusKnown::unknown;    // unknown (2)
            else if (unknown_factor > 0)                             // Some pixel are unknown
                pixel.status_known = Pixel::statusKnown::part_known; // paritally known (1) - else known (0) - see initialization value;
        }
    }
    // Compute remaining pixel batch for remaining resolutions (5x5, 9x9, 17x17, 33x33, ...)
    for (int l = 2, s = 2; l <= image_max_level_; ++l, (s <<= 1U)) {
        pooling_image_[l] = default_image;
        int s_half = s / 2;
#pragma omp parallel for
        for (int y = 0; y < image_height_; y++) {
            for (int x = 0; x < image_width_; x++) {
                Pixel& pixel = pooling_image_[l][x + y * image_width_];

                int left, right, top, bottom;
                if (x - s_half < 0) {
                    left = 0;
                    pixel.status_crossing = Pixel::statusCrossing::crossing; // crossing (1)
                }
                else {
                    left = x - s_half;
                }

                if ((x + s_half) >= image_width_) {
                    right = image_width_ - 1;
                    pixel.status_crossing = Pixel::statusCrossing::crossing; // crossing (1)
                }
                else {
                    right = x + s_half;
                }

                if (y - s_half < 0) {
                    top = 0;
                    pixel.status_crossing = Pixel::statusCrossing::crossing; // crossing (1)
                }
                else {
                    top = y - s_half;
                }

                if ((y + s_half) >= image_height_) {
                    bottom = image_height_ - 1;
                    pixel.status_crossing = Pixel::statusCrossing::crossing; // crossing (1)
                }
                else {
                    bottom = y + s_half;
                }
                /*
    Overlapping four neighbors (1-4) contain all information needed to find min/max of the parent batch (0)
    _____________________
    |        | |        |
    |        | |        |
    |    1   | |   2    |
    |________|_|________|
    |________|0|________|
    |        | |        |
    |    3   | |   4    |
    |        | |        |
    |________|_|________|

*/
                pixel.min = std::min(std::min(pooling_image_[l - 1][left + top * image_width_].min, pooling_image_[l - 1][right + top * image_width_].min),
                                     std::min(pooling_image_[l - 1][left + bottom * image_width_].min, pooling_image_[l - 1][right + bottom * image_width_].min));
                pixel.max = std::max(std::max(pooling_image_[l - 1][left + top * image_width_].max, pooling_image_[l - 1][right + top * image_width_].max),
                                     std::max(pooling_image_[l - 1][left + bottom * image_width_].max, pooling_image_[l - 1][right + bottom * image_width_].max));
                int unknown_factor = pooling_image_[l - 1][left + top * image_width_].status_known + pooling_image_[l - 1][right + top * image_width_].status_known
                    + pooling_image_[l - 1][left + bottom * image_width_].status_known + pooling_image_[l - 1][right + bottom * image_width_].status_known;
                if (unknown_factor == 8)                                 // All pixel are unknown -> 4 * unknown (2) = 8
                    pixel.status_known = Pixel::statusKnown::unknown;    // unknown (2)
                else if (unknown_factor > 0)                             // Some pixel are unknown
                    pixel.status_known = Pixel::statusKnown::part_known; // paritally known (1) - else known (0) - see initialization value;

                if (y >= s && y < image_height_ - s && x >= s && x < image_width_ - s) {
                    pixel.status_crossing = Pixel::statusCrossing::inside; // inside (0)
                }
            }
        }
    }

    // Find max value at by iterating through coarsest kernel pixel batches touching each other
    const int s = 1 << (image_max_level_ - 1);
    image_max_value_ = 0;
    for (int v = s; true; v += 2 * s) {
        if (v > image_height_ - s - 1) {
            v = image_height_ - s - 1;
        }
        for (int u = s; true; u += 2 * s) {
            if (u > image_width_ - s - 1) {
                u = image_width_ - s - 1;
            }
            int pixel_pos = u + image_width_ * v;
            Pixel pixel = pooling_image_[image_max_level_][pixel_pos];
            if (image_max_value_ < pixel.max) {
                image_max_value_ = pixel.max;
            }
            if (u == image_width_ - s - 1) {
                break;
            }
        }
        if (v == image_height_ - s - 1) {
            break;
        }
    }

    TOCK("image-construction")
}


template<>
inline Pixel DensePoolingImage<PinholeCamera>::poolBoundingBox(int u_min, int u_max, int v_min, int v_max) const
{
    const int u_diff = u_max - u_min;
    const int v_diff = v_max - v_min;

    const int min_side = std::min(u_diff, v_diff);

    int level = std::max((int) 0, std::min((int) image_max_level_, (int) (ceil(log2(min_side / 2)))));

    const int s = (level > 0) ? 1 << std::max((int) level - 1, 0) : 0;

    Pixel pixel_batch = Pixel::knownPixel();

    size_t count_pixel = 0;
    size_t count_unknown_pixel = 0;
    size_t count_partly_known_pixel = 0;

    int step = 2 * s + 1;

    for (int v = v_min + s; true; v += step) {
        if (v > v_max - s) {
            v = v_max - s;
        }
        for (int u = u_min + s; true; u += step) {
            if (u > u_max - s) {
                u = u_max - s;
            }

            int pixel_pos = u + image_width_ * v;
            Pixel pixel = pooling_image_[level][pixel_pos];

            if (pixel_batch.max < pixel.max) {
                pixel_batch.max = pixel.max;
            }
            if (pixel_batch.min > pixel.min) {
                pixel_batch.min = pixel.min;
            }

            if (pixel.status_known == 2) {
                count_unknown_pixel++;
                count_partly_known_pixel++;
            }
            else if (pixel.status_known == 1) {
                count_partly_known_pixel++;
            }

            count_pixel++;

            if (u == u_max - s) {
                break;
            }
        }
        if (v == v_max - s) {
            break;
        }
    }

    if (count_pixel == count_unknown_pixel) {
        pixel_batch.status_known = Pixel::statusKnown::unknown;
    }
    else if (count_partly_known_pixel > 0) {
        pixel_batch.status_known = Pixel::statusKnown::part_known;
    }

    return pixel_batch;
}


template<>
inline Pixel DensePoolingImage<PinholeCamera>::conservativeQuery(const Eigen::Vector2i& bb_min, const Eigen::Vector2i& bb_max) const
{
    bool u_in = true;   // << Pixel batch width is entirely contained within the image
    bool u_out = false; // << Pixel batch width is entirely outside the image

    int u_min = bb_min.x();
    int v_min = bb_min.y();
    int u_max = bb_max.x();
    int v_max = bb_max.y();

    if (u_min < Value(0)) {
        u_in = false;
        u_min = 0;
        if (u_max < Value(0))
            u_out = true;
    }
    else if (u_min > static_cast<Value>(image_width_ - 1U)) {
        u_out = true;
    }

    if (u_max > static_cast<Value>(image_width_ - 1U)) {
        u_in = false;
        u_max = static_cast<Value>(image_width_ - 1U);
    }

    bool v_in = true;   // << Pixel batch height is entirely contained within the image
    bool v_out = false; // << Pixel batch height is entirely outside the image

    if (v_min < Value(0)) {
        v_in = false;
        v_min = 0;
        if (v_max < Value(0))
            v_out = true;
    }
    else if (v_min > static_cast<Value>(image_height_ - 1U)) {
        v_out = true;
    }

    if (v_max > static_cast<Value>(image_height_ - 1U)) {
        v_in = false;
        v_max = static_cast<Value>(image_height_ - 1U);
    }

    // Check if the pixel batch is entirely outside the image
    if (u_out || v_out || u_min == u_max || v_min == v_max) {
        return Pixel::outsidePixelBatch();
    }

    Pixel pix = poolBoundingBox(u_min, u_max, v_min, v_max);

    // Check if the pixel batch is partly inside the image. Given the previous check this is equivalent to
    // not entirely inside or outside the image
    if (!u_in || !v_in) {
        pix.status_crossing = Pixel::statusCrossing::crossing;

        return pix;
    }
    else {
        return pix;
    }
}

} // namespace se

#endif // SE_DENSE_POOLING_IMAGE_HPP
