/*
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018 Nils Funk
 * SPDX-FileCopyrightText: 2019-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_IO_OCTOMAP_IO_IMPL_HPP
#define SE_MAP_IO_OCTOMAP_IO_IMPL_HPP

namespace se {

template<typename MapT, typename FunctionT>
octomap::OcTree* to_octomap(const MapT& map, FunctionT convert_data)
{
    static_assert(MapT::DataType::fld_ == Field::Occupancy, "se::to_octomap() is only implemented for occupancy maps");
    const float voxel_centre_offset = map.getRes() / 2.0f;

    // OcTree doesn't seem to have an API for setting data on any level except the finest so for
    // leaf Nodes and Blocks at scales higher than 0 all voxels they contain are iterated and set.
    octomap::OcTree* octomap = new octomap::OcTree(map.getRes());
    for (auto it = se::LeavesIterator(map.getOctree().get()); it != se::LeavesIterator<typename MapT::OctreeType>(); ++it) {
        const OctantBase* octant = *it;
        if (octant->isBlock()) {
            const auto& block = *static_cast<const typename MapT::OctreeType::BlockType*>(octant);
            const int current_scale = block.getCurrentScale();
            const Eigen::Vector3i block_coord = block.getCoord();
            for (unsigned z = 0; z < MapT::OctreeType::block_size; z++) {
                for (unsigned y = 0; y < MapT::OctreeType::block_size; y++) {
                    for (unsigned x = 0; x < MapT::OctreeType::block_size; x++) {
                        const Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(x, y, z);
                        Eigen::Vector3f point_W;
                        map.voxelToPoint(voxel_coord, point_W);
                        // The sample point is at the centre of voxels.
                        point_W += Eigen::Vector3f::Constant(voxel_centre_offset);
                        const octomap::point3d point_W_octomap(point_W.x(), point_W.y(), point_W.z());
                        const auto& data = block.getMaxData(voxel_coord, current_scale);
                        convert_data(*octomap, point_W_octomap, data);
                    }
                }
            }
        }
        else {
            const auto& node = *static_cast<const typename MapT::OctreeType::NodeType*>(octant);
            const auto& data = node.getMaxData();
            const Eigen::Vector3i node_coord = node.getCoord();
            for (int z = 0; z < node.getSize(); z++) {
                for (int y = 0; y < node.getSize(); y++) {
                    for (int x = 0; x < node.getSize(); x++) {
                        const Eigen::Vector3i voxel_coord = node_coord + Eigen::Vector3i(x, y, z);
                        Eigen::Vector3f point_W;
                        map.voxelToPoint(voxel_coord, point_W);
                        // The sample point is at the centre of voxels.
                        point_W += Eigen::Vector3f::Constant(voxel_centre_offset);
                        const octomap::point3d point_W_octomap(point_W.x(), point_W.y(), point_W.z());
                        convert_data(*octomap, point_W_octomap, data);
                    }
                }
            }
        }
    }

    // Make the octree consistent at all levels.
    octomap->updateInnerOccupancy();
    // Combine children with the same values.
    octomap->prune();
    return octomap;
}


template<typename MapT>
octomap::OcTree* to_octomap(const MapT& map)
{
    return to_octomap(map, [](octomap::OcTree& octomap, const octomap::point3d& point_W, const typename MapT::DataType& data) -> void {
        // Do not store log-odds of 0 because OctoMap interprets it as occupied,
        // whereas in supereight it means unknown.
        if (is_valid(data)) {
            const float occupancy = get_field(data) * data.weight;
            octomap.setNodeValue(point_W, occupancy, true);
        }
    });
}


template<typename MapT>
octomap::OcTree* to_binary_octomap(const MapT& map)
{
    return to_octomap(map, [](octomap::OcTree& octomap, const octomap::point3d& point_W, const typename MapT::DataType& data) -> void {
        const float occupancy = get_field(data) * data.weight;
        if (occupancy > 0.0f) {
            // Occupied
            octomap.updateNode(point_W, true, true);
        }
        else if (occupancy < 0.0f) {
            // Free
            octomap.updateNode(point_W, false, true);
        }
        // Do not update unknown voxels.
    });
}

} // namespace se

#endif // SE_MAP_IO_OCTOMAP_IO_IMPL_HPP
