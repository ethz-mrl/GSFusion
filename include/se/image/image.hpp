/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_HPP
#define SE_IMAGE_HPP

#include <Eigen/StdVector>
#include <cassert>

#include "se/common/colour_types.hpp"
#include "se/common/colour_utils.hpp"

namespace se {

template<typename T>
class Image {
    public:
    Image(const unsigned w, const unsigned h) : width_(w), height_(h), data_(width_ * height_)
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const unsigned w, const unsigned h, const T& val) : width_(w), height_(h)
    {
        assert(width_ > 0 && height_ > 0);
        data_.resize(width_ * height_, val);
    }

    T& operator[](std::size_t idx)
    {
        return data_[idx];
    }
    const T& operator[](std::size_t idx) const
    {
        return data_[idx];
    }

    T& operator()(const int x, const int y)
    {
        return data_[x + y * width_];
    }
    const T& operator()(const int x, const int y) const
    {
        return data_[x + y * width_];
    }

    std::size_t size() const
    {
        return width_ * height_;
    };
    int width() const
    {
        return width_;
    };
    int height() const
    {
        return height_;
    };

    T* data()
    {
        return data_.data();
    }
    const T* data() const
    {
        return data_.data();
    }

    private:
    int width_;
    int height_;
    std::vector<T, Eigen::aligned_allocator<T>> data_;

    // std::vector<bool> is specialized for space efficiency which means that element access doesn't
    // return references to the data as expected, causing compilation issues.
    static_assert(!std::is_same<T, bool>::value, "Use char/uint8_t instead of bool to avoid the std::vector<bool> specialization");
};


template<typename T>
void remap(const Image<T>& input, Image<T>& output, const Image<Eigen::Vector2i>& map)
{
    assert((output.width() == map.width()) && "The output and map have the same width");
    assert((output.height() == map.height()) && "The output and map have the same height");
#pragma omp parallel for
    for (size_t i = 0; i < output.size(); i++) {
        const Eigen::Vector2i& idx = map[i];
        output[i] = input(idx.x(), idx.y());
    }
}


static void convert_to_output_colour_img(const Image<rgb_t>& input_colour_img, uint32_t* output_colour_img_data)
{
    rgb_to_rgba(reinterpret_cast<const uint8_t*>(input_colour_img.data()), output_colour_img_data, input_colour_img.width() * input_colour_img.height());
}


static void convert_to_output_depth_img(const se::Image<float>& input_depth_img, uint32_t* output_depth_img_data)
{
    depth_to_rgba(output_depth_img_data, input_depth_img.data(), Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()), 0, std::numeric_limits<float>::max());
}


static void convert_to_output_depth_img(const se::Image<float>& input_depth_img, const float min_depth, const float max_depth, uint32_t* output_depth_img_data)
{
    depth_to_rgba(output_depth_img_data, input_depth_img.data(), Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()), min_depth, max_depth);
}


} // end namespace se

#endif // SE_IMAGE_HPP
