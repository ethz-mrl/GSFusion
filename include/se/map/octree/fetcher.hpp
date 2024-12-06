/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_FETCHER_HPP
#define SE_FETCHER_HPP

#include <vector>

#include "octree.hpp"
#include "se/map/utils/type_util.hpp"


namespace se {
namespace fetcher {

/**
 * \brief Fetch the octant for given coordinates and scale.
 *
 * \tparam OctreeT
 * \param octant_coord    The coordinates of the octant to be fetched
 * \param scale_desired   The scale of the node to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the octant if it is allocated, nullptr otherwise
 */
template<typename OctreeT>
inline se::OctantBase* octant(const Eigen::Vector3i& octant_coord, const se::scale_t scale_desired, se::OctantBase* base_parent_ptr);

/**
 * \brief Same as se::fetcher::octant() but returning a const pointer.
 */
template<typename OctreeT>
inline const se::OctantBase* octant(const Eigen::Vector3i& octant_coord, const se::scale_t scale_desired, const se::OctantBase* base_parent_ptr);

/**
 * \brief Fetch the octant for given coordinates and scale.
 *        Returnes the finest allocated octant up to the desired scale
 *
 * \tparam OctreeT
 * \param octant_coord    The coordinates of the octant to be fetched
 * \param scale_desired   The scale of the node to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the finest allocated octant up to the desired scale
 */
template<typename OctreeT>
inline se::OctantBase* finest_octant(const Eigen::Vector3i& octant_coord, const se::scale_t scale_desired, se::OctantBase* base_parent_ptr);

/**
 * \brief Same as se::fetcher::finest_octant() but returning a const pointer.
 */
template<typename OctreeT>
inline const se::OctantBase* finest_octant(const Eigen::Vector3i& octant_coord, const se::scale_t scale_desired, const se::OctantBase* base_parent_ptr);

/**
 * \brief Fetch the block for given block coordinates.
 *
 * \tparam OctreeT
 * \param block_coord     The coordinates of the block to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the block if allocated, nullptr otherwise
 */
template<typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& block_coord, se::OctantBase* base_parent_ptr);

/**
 * \brief Fetch the leaf for given block coordinates.
 *
 * \tparam OctreeT
 * \param leaf_coord      The coordinates of the block to be fetched
 * \param base_parent_ptr The parent pointer to start the fetching process from (e.g. the octrees root)
 *
 * \return The pointer to the leaf
 */
template<typename OctreeT>
inline OctantBase* leaf(const Eigen::Vector3i& leaf_coord, OctantBase* base_parent_ptr);

/**
 * \brief Same as se::fetcher::leaf() but returning a const pointer.
 */
template<typename OctreeT>
inline const OctantBase* leaf(const Eigen::Vector3i& leaf_coord, const OctantBase* base_parent_ptr);

inline const Eigen::Matrix<int, 3, 6>& face_neighbour_offsets();

template<typename OctreeT>
inline std::vector<const OctantBase*> face_neighbours(const OctantBase* octant, const OctantBase* common_parent, const OctreeT& octree);

} // namespace fetcher
} // namespace se

#include "impl/fetcher_impl.hpp"

#endif // SE_FETCHER_HPP
