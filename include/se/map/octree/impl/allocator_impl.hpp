/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ALLOCATOR_IMPL_HPP
#define SE_ALLOCATOR_IMPL_HPP

namespace se {
namespace allocator {

template<typename OctreeT>
inline se::OctantBase* block(const Eigen::Vector3i& voxel_coord, OctreeT& octree, OctantBase* base_parent_ptr)
{
    se::key_t voxel_key;
    se::keyops::encode_key(voxel_coord, 0, voxel_key); // Allocate up to finest scale

    return block(voxel_key, octree, base_parent_ptr);
}


template<typename OctreeT>
inline se::OctantBase* block(const se::key_t voxel_key, OctreeT& octree, OctantBase* base_parent_ptr)
{
    assert(base_parent_ptr); // Verify parent ptr
    //  assert(se::keyops::is_child(base_parent_ptr->getKey(), voxel_key)); // TODO:
    assert(se::keyops::key_to_scale(voxel_key) <= octree.max_block_scale); // Verify scale is within block

    se::OctantBase* child_ptr;
    se::allocator::detail::allocate_key(voxel_key, octree, base_parent_ptr, child_ptr);
    return child_ptr;
}


template<typename OctreeT>
inline std::vector<se::OctantBase*>
blocks(const std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords, OctreeT& octree, se::OctantBase* base_parent_ptr, const bool only_allocated)
{
    std::set<se::key_t> voxel_key_set;

#pragma omp declare reduction(merge : std::set <se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge : voxel_key_set)
    for (unsigned int i = 0; i < voxel_coords.size(); i++) {
        const Eigen::Vector3i voxel_coord = voxel_coords[i];
        se::key_t voxel_key;
        se::keyops::encode_key(voxel_coord, octree.max_block_scale, voxel_key);
        voxel_key_set.insert(voxel_key);
    }

    std::vector<se::key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());

    return blocks(voxel_keys, octree, base_parent_ptr, only_allocated);
}


template<typename OctreeT>
inline std::vector<se::OctantBase*> blocks(std::vector<se::key_t>& unique_voxel_keys, OctreeT& octree, se::OctantBase* base_parent_ptr, const bool only_allocated)
{
    assert(base_parent_ptr); // Verify parent ptr

    // Allocate Nodes up to block_scale. It is important that the Nodes are allocated here and not
    // during the Block allocation below. Since Block allocation happens in parallel, it is possible
    // for two Blocks to allocate the same parent twice. Doing the allocation scale-by-scale here
    // prevents this problem.
    for (scale_t scale = octree.getMaxScale(); scale > octree.max_block_scale; scale--) {
        std::vector<se::key_t> unique_voxel_keys_at_scale;
        se::keyops::unique_at_scale(unique_voxel_keys, scale, unique_voxel_keys_at_scale);

#pragma omp parallel for
        for (unsigned int i = 0; i < unique_voxel_keys_at_scale.size(); i++) {
            const auto unique_voxel_key_at_scale = unique_voxel_keys_at_scale[i];
            // We don't care what the address of the allocated Node is, just that it's allocated.
            se::OctantBase* unused_result;
            se::allocator::detail::allocate_key(unique_voxel_key_at_scale, octree, base_parent_ptr, unused_result);
        }
    }

    // Allocate blocks and store block pointers
    std::vector<se::OctantBase*> block_ptrs;
#pragma omp parallel for
    for (unsigned int i = 0; i < unique_voxel_keys.size(); i++) {
        const auto unique_voxel_key = unique_voxel_keys[i];
        assert(se::keyops::key_to_scale(unique_voxel_key) <= octree.max_block_scale); // Verify scale is within block

        se::OctantBase* child_ptr;
        const bool did_allocation = se::allocator::detail::allocate_key(unique_voxel_key, octree, base_parent_ptr, child_ptr);

        // Don't push back if only the newly allocated Octants are returned and no allocation happened.
        if (!(only_allocated && !did_allocation)) {
#pragma omp critical
            {
                block_ptrs.push_back(child_ptr);
            }
        }
    }

    return block_ptrs;
}


namespace detail {


template<typename OctreeT>
inline bool allocate_key(const se::key_t key, OctreeT& octree, se::OctantBase* base_parent_ptr, se::OctantBase*& allocated_octant)
{
    assert(se::keyops::is_valid(key)); // Verify if the key is valid
    typename OctreeT::NodeType* parent_ptr = static_cast<typename OctreeT::NodeType*>(base_parent_ptr);
    se::OctantBase* child_ptr = nullptr;

    int child_scale = se::math::log2_const(parent_ptr->getSize()) - 1;
    se::code_t code = se::keyops::key_to_code(key);
    int min_scale = std::max(se::keyops::key_to_scale(key), octree.max_block_scale);

    bool did_allocation = false;
    for (; child_scale >= min_scale; --child_scale) {
        se::idx_t child_idx = se::keyops::code_to_child_idx(code, child_scale);
        did_allocation = octree.allocate(parent_ptr, child_idx, child_ptr);
        parent_ptr = static_cast<typename OctreeT::NodeType*>(child_ptr);
    }
    allocated_octant = child_ptr;
    return did_allocation;
}


} // namespace detail
} // namespace allocator
} // namespace se

#endif // SE_ALLOCATOR_IMPL_HPP
