/*
 * SPDX-FileCopyrightText: 2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PROPAGATOR_HPP
#define SE_PROPAGATOR_HPP

#include <unordered_set>

#include "se/common/timings.hpp"
#include "se/map/octant/octant.hpp"
#include "se/map/utils/type_util.hpp"

namespace se {


// Forward decleration
class OctantBase;

namespace propagator {


/**
 * \brief Propagate the block values from the current scale to a lower target scale.
 *
 * \tparam OctreeT
 * \tparam ChildF
 * \tparam ParentF
 * \param[in] octree        The octree containing the block
 * \param[in] octant_ptr    The pointer to the blocks octant base
 * \param[in] init_scale    The scale to start the propagation from (usually current scale)
 * \param[in] child_funct   The function for the child update
 * \param[in] parent_funct  The function for the parent update
 */
template<typename OctreeT, typename ChildF, typename ParentF>
void propagateBlockUp(const OctreeT& /* octree */, se::OctantBase* octant_ptr, const int init_scale, ChildF child_funct, ParentF parent_funct);

/**
 * \brief Propagate the block values from the current scale to a lower target scale.
 *
 * \tparam OctreeT
 * \tparam ChildF
 * \tparam ParentF
 * \param[in] octree        The octree containing the block
 * \param[in] octant_ptr    The pointer to the blocks octant base
 * \param[in] target_scale  The scale to propagate the values down
 * \param[in] child_funct   The function for the child update
 * \param[in] parent_funct  The function for the parent update
 */
template<typename OctreeT, typename ChildF, typename ParentF>
void propagateBlockDown(const OctreeT& octree, se::OctantBase* octant_ptr, const int target_scale, ChildF child_funct, ParentF parent_funct);

/**
 * \brief Propagate all nodes to the root using a given up-propagation function.
 *
 * \tparam PropagateF
 * \param[in] octant_ptrs       The pointers to the leaf nodes.
 * \param[in] propagate_funct   The function used for the up-propagation
 */
template<typename PropagateF>
void propagateBlocksToRoot(std::vector<se::OctantBase*>& octant_ptrs, PropagateF& propagate_funct);

/**
 * \brief Propagate all node time stamps to the root.
 *
 * \tparam PropagateF
 * \param[in] octant_ptrs   The pointers to the leaf nodes.
 */
void propagateBlockTimeStampsToRoot(std::vector<se::OctantBase*>& octant_ptrs);


} // namespace propagator
} // namespace se

#include "impl/propagator_impl.hpp"

#endif // SE_PROPAGATOR_HPP
