/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SETUP_UTIL_HPP
#define SE_SETUP_UTIL_HPP

#include <Eigen/Core>

#include "se/common/semantic_classes.hpp"

namespace se {

static inline Eigen::Vector3f sample_offset_frac = Eigen::Vector3f::Constant(0.5f);

// Representation enums
enum class Field { TSDF, Occupancy };
enum class Colour { On = true, Off = false };
/** \brief The enum used to select the semantic class set used in mapping.
 * The value of each enumerator is the number of semantic classes including the background.
 */
enum class Semantics : int { Off = 0, NYUv2 = classes::nyuv2.size(), MP3D = classes::mp3d.size(), COCO = classes::coco.size(), Replica = classes::replica.size() };

// Other enums
enum class Res { Single, Multi };
enum class Integ { Simple, PinholeCamera };
enum class Safe { On = true, Off = false }; // Switch between Safe and Sorry

enum class AllocMeth { Raycasting, VoxelCarving }; // Allocation method
enum class Rep { Surface, Freespace };             // Map representation

/**
 *  \brief The enum classes to define the sorting templates
 */
enum class Sort { SmallToLarge, LargeToSmall };

/**
 * \brief helper class for static_cast removal. See crtp-static-cast-replacement-*.png for inherintance.
 *        See: https://www.fluentcpp.com/2017/05/19/crtp-helper/
 *
 * \tparam crtpType This avoids the diamond shape if two classes are the Base of the same T.
 *                  This way every crtp class unique.
 * \tparam T        The type of the Derived class
 */
template<typename crtpType, typename T>
struct crtp {
    T& underlying()
    {
        return static_cast<T&>(*this);
    }
    const T& underlying() const
    {
        return static_cast<const T&>(*this);
    }

    private:
    crtp()
    {
    } ///< private-constructor-and-friend technique
    friend crtpType;
};

} // namespace se

#endif // SE_SETUP_UTIL_HPP
