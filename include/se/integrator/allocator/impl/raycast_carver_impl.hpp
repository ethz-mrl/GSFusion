/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCAST_CARVER_IMPL_HPP
#define SE_RAYCAST_CARVER_IMPL_HPP

namespace se {
namespace fetcher {


template<typename MapT, typename SensorT>
inline std::vector<se::OctantBase*> frustum(MapT& map, const SensorT& sensor, const Eigen::Matrix4f& T_WS)
{
    const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_WS);
    // Loop over all allocated Blocks.
    std::vector<se::OctantBase*> fetched_block_ptrs;

    for (auto block_ptr_itr = se::FrustumIterator<MapT, SensorT>(map, sensor, T_SM); block_ptr_itr != se::FrustumIterator<MapT, SensorT>(); ++block_ptr_itr) {
        fetched_block_ptrs.push_back(*block_ptr_itr);
    }

    return fetched_block_ptrs;
}


} // namespace fetcher

template<typename MapT, typename SensorT>
RaycastCarver<MapT, SensorT>::RaycastCarver(MapT& map, const SensorT& sensor, const se::Image<float>& depth_img, const Eigen::Matrix4f& T_WS, const int frame) :
        map_(map), octree_(*(map_.getOctree())), sensor_(sensor), depth_img_(depth_img), T_WS_(T_WS), frame_(frame), config_(map)
{
}


template<typename MapT, typename SensorT>
std::vector<se::OctantBase*> RaycastCarver<MapT, SensorT>::operator()()
{
    TICK("fetch-frustum")
    // Fetch the currently allocated Blocks in the sensor frustum.
    // i.e. the fetched blocks might contain blocks outside the current valid sensor range.
    std::vector<se::OctantBase*> fetched_block_ptrs = se::fetcher::frustum(map_, sensor_, T_WS_);
    TOCK("fetch-frustum")

    TICK("create-list")
    se::OctantBase* root_ptr = octree_.getRoot();

    const int num_steps = ceil(config_.band / (2 * map_.getRes()));

    const Eigen::Vector3f t_WS = T_WS_.topRightCorner<3, 1>();

#pragma omp declare reduction(merge : std::set <se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
    std::set<se::key_t> voxel_key_set;

#pragma omp parallel for reduction(merge : voxel_key_set)
    for (int x = 0; x < depth_img_.width(); ++x) {
        for (int y = 0; y < depth_img_.height(); ++y) {
            const Eigen::Vector2i pixel(x, y);
            const float depth_value = depth_img_(pixel.x(), pixel.y());
            // Only consider depth values inside the valid sensor range
            if (depth_value < sensor_.near_plane || depth_value > (sensor_.far_plane + config_.band * 0.5f)) {
                continue;
            }

            Eigen::Vector3f ray_dir_C;
            const Eigen::Vector2f pixel_f = pixel.cast<float>();
            sensor_.model.backProject(pixel_f, &ray_dir_C);
            const Eigen::Vector3f point_W = (T_WS_ * (depth_value * ray_dir_C).homogeneous()).template head<3>();

            const Eigen::Vector3f reverse_ray_dir_W = (t_WS - point_W).normalized();

            const Eigen::Vector3f ray_origin_W = point_W - (config_.band * 0.5f) * reverse_ray_dir_W;
            const Eigen::Vector3f step = (reverse_ray_dir_W * config_.band) / num_steps;

            Eigen::Vector3f ray_pos_W = ray_origin_W;
            for (int i = 0; i < num_steps; i++) {
                Eigen::Vector3i voxel_coord;

                if (map_.template pointToVoxel<se::Safe::On>(ray_pos_W, voxel_coord)) {
                    const se::OctantBase* octant_ptr = se::fetcher::block<typename MapT::OctreeType>(voxel_coord, root_ptr);
                    if (octant_ptr == nullptr) {
                        se::key_t voxel_key;
                        se::keyops::encode_key(voxel_coord, octree_.max_block_scale, voxel_key);
                        voxel_key_set.insert(voxel_key);
                    }
                }
                ray_pos_W += step;
            }
        }
    }
    // Allocate the Blocks and get pointers only to the newly-allocated Blocks.
    std::vector<key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());
    TOCK("create-list")

    TICK("allocate-list")
    std::vector<se::OctantBase*> allocated_block_ptrs = se::allocator::blocks(voxel_keys, octree_, octree_.getRoot(), true);
    TOCK("allocate-list")

    TICK("combine-vectors")
    // Merge the previously-allocated and newly-allocated Block pointers.
    allocated_block_ptrs.reserve(allocated_block_ptrs.size() + fetched_block_ptrs.size());
    allocated_block_ptrs.insert(allocated_block_ptrs.end(), fetched_block_ptrs.begin(), fetched_block_ptrs.end());
    TOCK("combine-vectors")
    return allocated_block_ptrs;
}


} // namespace se

#endif // SE_RAYCAST_CARVER_IMPL_HPP
