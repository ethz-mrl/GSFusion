/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_INTEGRATOR_IMPL_HPP
#define SE_INTEGRATOR_IMPL_HPP

namespace se {
namespace integrator {

template<typename OctreeT>
inline bool setData(OctreeT& octree, const Eigen::Vector3i& voxel_coord, const typename OctreeT::DataType& data)
{
    se::OctantBase* octant_ptr = octree.getRoot();

    if (!octant_ptr) // Return false if the octree root isn't allocated
    {
        std::cerr << "Re-initalise octree. Root missing" << std::endl;
        return false;
    }

    unsigned int node_size = octree.getSize();
    for (; node_size >= OctreeT::block_size; node_size = node_size >> 1) {
        typename OctreeT::NodeType* node_ptr = std::static_pointer_cast<typename OctreeT::NodeType>(octant_ptr);
        se::OctantBase* octant_tmp_ptr = nullptr;
        const int child_idx = get_child_idx(voxel_coord, node_ptr);
        if (!node_ptr->getChild(child_idx, octant_tmp_ptr)) {
            octant_ptr = std::static_pointer_cast<OctantBase>(se::allocator::block(voxel_coord, octree, node_ptr));
            break;
        }
        octant_ptr = octant_tmp_ptr;
    }

    std::static_pointer_cast<typename OctreeT::BlockType>(octant_ptr)->setData(voxel_coord, data);

    return true;
}

} // namespace integrator
} // namespace se

#endif // SE_INTEGRATOR_IMPL_HPP
