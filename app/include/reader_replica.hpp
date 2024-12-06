/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: MIT
 */

#ifndef __READER_REPLICA_HPP
#define __READER_REPLICA_HPP


#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <string>

#include "reader_base.hpp"
#include "se/image/image.hpp"


namespace se {

/** Reader for the Replica dataset. */
class ReplicaReader : public Reader {
    public:
    /** Construct a ReplicaReader from a ReaderConfig.
     *
     * \param[in] c The configuration struct to use.
     */
    ReplicaReader(const ReaderConfig& c);


    /** Restart reading from the beginning. */
    void restart();


    /** The name of the reader.
     *
     * \return The string `"ReplicaReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    static constexpr float replica_inverse_scale_ = 1.0f / 6553.5f;
    float inverse_scale_;

    std::vector<std::string> depth_filenames_;

    std::vector<std::string> rgb_filenames_;

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextColour(Image<rgb_t>& colour_image);
};

} // namespace se


#endif