/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_INTEGRATOR_HPP
#define SE_INTEGRATOR_HPP

#include "allocator.hpp"

namespace se {

/**
 * Helper wrapper to integrate data in the octree.
 */
namespace integrator {

/**
 * \brief Set voxel data for a given coordinate
 *
 * \tparam OctreeT          The type of the octree used
 * \param[in] octree_ptr    The shared pointer to the octree
 * \param[in] voxel_coord   The coordinates of the voxel to be set
 * \param[in] data          The data to be set in the voxel
 *
 * \return True if the data was set, False otherwise
 */
template<typename OctreeT>
inline bool setData(OctreeT& octree, const Eigen::Vector3i& voxel_coord, const typename OctreeT::DataType& data);

} // namespace integrator
} // namespace se

#include "impl/integrator_impl.hpp"

#endif // SE_INTEGRATOR_HPP
