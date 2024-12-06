/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ALLOCATOR_HPP
#define SE_ALLOCATOR_HPP

#include <Eigen/StdVector>
#include <set>

#include "octree.hpp"
#include "se/map/utils/type_util.hpp"

/**
 * Helper wrapper to allocate and de-allocate octants in the octree.
 * The actual allocation and deallocation of memory is still only handled by the octree class.
 */
namespace se {
namespace allocator {

/**
 * \brief Allocate a block at the provided voxel coordinates.
 *
 * \tparam OctreeT              The Octree template
 * \param[in] voxel_coord       The 3D coordinates of a voxel within the block
 * \param[in] octree        The octree to allocate the block in
 * \param[in] base_parent_ptr   The starting node pointer (default nullptr will be replaced with octree root)
 *
 * \return A block pointer to the allocated block
 */
template<typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& voxel_coord, OctreeT& octree, se::OctantBase* base_parent_ptr);

template<typename OctreeT>
inline se::OctantBase* block(const se::key_t voxel_key, OctreeT& octree, se::OctantBase* base_parent_ptr);

/**
 * \brief Allocate Blocks at the provided voxel coordinates.
 *
 * \tparam OctreeT            The Octree template.
 * \param[in] voxel_coords    The 3D coordinates of a voxel within each block.
 * \param[in] octree          The octree to allocate the blocks in.
 * \param[in] base_parent_ptr The starting node pointer. A nullptr will be replaced with the octree
 *                            root.
 * \param[in] only_allocated  Return pointers only for the newly allocated Blocks instead of all the
 *                            Blocks corresponding to the coordinates in voxel_coords.
 *
 * \return Pointers to the allocated Octants.
 */
template<typename OctreeT>
std::vector<se::OctantBase*>
blocks(const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords, OctreeT& octree, se::OctantBase* base_parent_ptr, const bool only_allocated = false);

/**
 * \brief Allocate Blocks at the provided voxel Morton codes.
 *
 * \tparam OctreeT            The Octree template.
 * \param[in] voxel_keys      The Morton code of a voxel within each block.
 * \param[in] octree          The octree to allocate the blocks in.
 * \param[in] base_parent_ptr The starting node pointer. A nullptr will be replaced with the octree
 *                            root.
 * \param[in] only_allocated  Return pointers only for the newly allocated Blocks instead of all the
 *                            Blocks corresponding to the Morton codes in voxel_keys.
 *
 * \return Pointers to the allocated Octants.
 */
template<typename OctreeT>
std::vector<se::OctantBase*> blocks(std::vector<se::key_t>& voxel_keys, OctreeT& octree, se::OctantBase* base_parent_ptr, const bool only_allocated = false);

namespace detail {


/**
 * \brief Allocate a given key in the octree.
 *
 * \warning Initialises each child with it's parents value.
 *          I.e. default value for TSDF but might differ for occupancy.
 *
 * \tparam OctreeT
 * \param[in] key               The key to be allocated
 * \param[in] octree            The octree to allocate the key in
 * \param[in] base_parent_ptr   The base pointer to start the octree traversal from (e.g. octree.getRoot())
 * \param[in] allocated_octant  The pointer to the allocated octant
 *
 * \return True if the octant has been allocated, false if it already was
 */
template<typename OctreeT>
inline bool allocate_key(const se::key_t key, OctreeT& octree, se::OctantBase* base_parent_ptr, se::OctantBase*& allocated_octant);


} // namespace detail
} // namespace allocator
} // namespace se

#include "impl/allocator_impl.hpp"

#endif // SE_ALLOCATOR_HPP
