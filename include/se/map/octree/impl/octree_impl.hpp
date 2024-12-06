/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_IMPL_HPP
#define SE_OCTREE_IMPL_HPP

namespace se {

template<typename DataT, Res ResT, int BlockSize>
Octree<DataT, ResT, BlockSize>::Octree(const int size) : size_(math::power_two_up(std::max(size, 2 * BlockSize))), root_ptr_(memory_pool_.allocateRoot(Eigen::Vector3i::Zero(), size_))
{
}


template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::begin()
{
    return OctreeIterator<Octree<DataT, ResT, BlockSize>>(this);
}


template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::begin() const
{
    return cbegin();
}


template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::cbegin() const
{
    return OctreeIterator<const Octree<DataT, ResT, BlockSize>>(this);
}


template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::end()
{
    return OctreeIterator<Octree<DataT, ResT, BlockSize>>();
}


template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::end() const
{
    return cend();
}


template<typename DataT, Res ResT, int BlockSize>
OctreeIterator<const Octree<DataT, ResT, BlockSize>> Octree<DataT, ResT, BlockSize>::cend() const
{
    return OctreeIterator<const Octree<DataT, ResT, BlockSize>>();
}


template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::contains(const Eigen::Vector3i& voxel_coord) const
{
    return voxel_coord.x() >= 0 && voxel_coord.x() < size_ && voxel_coord.y() >= 0 && voxel_coord.y() < size_ && voxel_coord.z() >= 0 && voxel_coord.z() < size_;
}


template<typename DataT, Res ResT, int BlockSize>
bool Octree<DataT, ResT, BlockSize>::allocate(NodeType* parent_ptr, const int child_idx, OctantBase*& child_ptr)
{
    assert(parent_ptr);
    assert(!parent_ptr->isBlock());

    child_ptr = parent_ptr->getChild(child_idx);
    if (child_ptr) {
        return false;
    }

    const DataT& init_data = parent_ptr->getData();
    if (parent_ptr->getSize() == 2 * BlockSize) {
#pragma omp critical
        {
            child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx, init_data);
        }
        aabbExtend(child_ptr->getCoord(), parent_ptr->getSize() / 2);
    }
    else {
#pragma omp critical
        {
            child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx, init_data);
        }
    }
    parent_ptr->setChild(child_idx, child_ptr);
    return true;
}


template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::allocateChildren(NodeType* parent_ptr)
{
    assert(parent_ptr);
    const DataT& init_data = parent_ptr->getData();
    const bool children_are_blocks = parent_ptr->getSize() == 2 * BlockSize;
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        OctantBase* child_ptr = parent_ptr->getChild(child_idx);
        if (child_ptr) {
            continue;
        }
        if (children_are_blocks) {
#pragma omp critical
            {
                child_ptr = memory_pool_.allocateBlock(parent_ptr, child_idx, init_data);
            }
        }
        else {
#pragma omp critical
            {
                child_ptr = memory_pool_.allocateNode(parent_ptr, child_idx, init_data);
            }
        }
        parent_ptr->setChild(child_idx, child_ptr);
    }
    if (children_are_blocks) {
        // Blocks are always leaves and all of them have been allocated, extend the AABB to contain
        // their parent.
        aabbExtend(parent_ptr->getCoord(), parent_ptr->getSize());
    }
}


template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::deleteChildren(NodeType* parent_ptr)
{
    for (int child_idx = 0; child_idx < 8; child_idx++) {
        OctantBase* child_ptr = parent_ptr->getChild(child_idx);
        if (child_ptr) {
            if (child_ptr->isBlock()) {
                memory_pool_.deleteBlock(static_cast<BlockType*>(child_ptr));
            }
            else {
                NodeType* node_ptr = static_cast<NodeType*>(child_ptr);
                deleteChildren(node_ptr);
                memory_pool_.deleteNode(node_ptr);
            }
        }
        parent_ptr->setChild(child_idx, nullptr);
    }
    parent_ptr->clearChildrenMask();
}


template<typename DataT, Res ResT, int BlockSize>
const Eigen::AlignedBox3i& Octree<DataT, ResT, BlockSize>::aabb() const
{
    return aabb_;
}


template<typename DataT, Res ResT, int BlockSize>
void Octree<DataT, ResT, BlockSize>::aabbExtend(const Eigen::Vector3i& voxel_coord, const int size)
{
    const Eigen::AlignedBox3i octant_aabb(voxel_coord, voxel_coord + Eigen::Vector3i::Constant(size));
    aabb_.extend(octant_aabb);
}

} // namespace se

#endif // SE_OCTREE_IMPL_HPP
