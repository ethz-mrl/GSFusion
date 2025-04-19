/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __READER_TUM_HPP
#define __READER_TUM_HPP


#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <string>

#include "reader_base.hpp"
#include "se/image/image.hpp"


namespace se {

/** Reader for the TUM RGBD dataset.
  * https://vision.in.tum.de/data/datasets/rgbd-dataset
  */
class TUMReader : public Reader {
    public:
    /** Construct a TUMReader from a ReaderConfig.
      *
      * \param[in] c The configuration struct to use.
      */
    TUMReader(const ReaderConfig& c);


    /** Restart reading from the beginning. */
    void restart();


    /** The name of the reader.
      *
      * \return The string `"TUMReader"`.
      */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    static constexpr float tum_inverse_scale_ = 1.0f / 5000.0f;
    float inverse_scale_;

    static constexpr double max_match_timestamp_dist_ = 0.02;

    static constexpr double max_interp_timestamp_dist_ = 10.0 * max_match_timestamp_dist_;

    std::vector<std::string> depth_filenames_;

    std::vector<std::string> rgb_filenames_;

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextColour(Image<rgb_t>& colour_image);
};

} // namespace se


#endif