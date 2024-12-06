/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VOXEL_CARVER_IMPL_HPP
#define SE_VOXEL_CARVER_IMPL_HPP


namespace se {


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::VolumeCarver(MapType& map,
                                                                                                            const SensorT& sensor,
                                                                                                            const se::Image<float>& depth_img,
                                                                                                            const Eigen::Matrix4f& T_WS,
                                                                                                            const int frame) :
        map_(map),
        octree_(*(map.getOctree())),
        sensor_(sensor),
        depth_pooling_img_(depth_img),
        T_SW_(se::math::to_inverse_transformation(T_WS)),
        frame_(frame),
        map_res_(map.getRes()),
        config_(map),
        max_depth_value_(std::min(sensor.far_plane, depth_pooling_img_.maxValue() + config_.tau_max)),
        zero_depth_band_(1.0e-6f),
        size_to_radius_(std::sqrt(3.0f) / 2.0f)
{
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
VolumeCarverAllocation VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::operator()()
{
    NodeType* root_ptr = static_cast<NodeType*>(octree_.getRoot());
    const int child_size = root_ptr->getSize() / 2;
    octree_.allocateChildren(root_ptr);
    // Launch on the root node's children.
#pragma omp parallel for
    for (int child_idx = 0; child_idx < 8; ++child_idx) {
        (*this)(root_ptr->getChildCoord(child_idx), child_size, 1, root_ptr->getChild(child_idx));
    }

    // Extend the octree AABB to contain all leaf nodes. See se::Octree::aabbExtend() on why this
    // can't be done on the octree side.
    for (const OctantBase* octant : allocation_list_.node_list) {
        if (octant->isLeaf()) {
            octree_.aabbExtend(octant->getCoord(), static_cast<const NodeType*>(octant)->getSize());
        }
    }

    return allocation_list_;
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
bool VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::crossesFrustum(std::vector<srl::projection::ProjectionStatus>& proj_corner_stati)
{
    for (int corner_idx = 0; corner_idx < 8; corner_idx++) {
        if (proj_corner_stati[corner_idx] == srl::projection::ProjectionStatus::Successful) {
            return true;
        }
    }
    return false;
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
bool VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::cameraInNode(const Eigen::Vector3i& node_coord, const int node_size, const Eigen::Matrix4f& T_WS)
{
    Eigen::Vector3f voxel_coord_f;
    map_.pointToVoxel(se::math::to_translation(T_WS), voxel_coord_f);
    if (voxel_coord_f.x() >= node_coord.x() && voxel_coord_f.x() <= node_coord.x() + node_size && voxel_coord_f.y() >= node_coord.y() && voxel_coord_f.y() <= node_coord.y() + node_size
        && voxel_coord_f.z() >= node_coord.z() && voxel_coord_f.z() <= node_coord.z() + node_size) {
        return true;
    }
    return false;
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
se::VarianceState VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::computeVariance(const float depth_value_min,
                                                                                                                                 const float depth_value_max,
                                                                                                                                 const float node_dist_min_m,
                                                                                                                                 const float node_dist_max_m)
{
    // Assume worst case scenario -> no multiplication with proj_scale
    const float z_diff_max = (node_dist_max_m - depth_value_min); // * proj_scale;
    const float z_diff_min = (node_dist_min_m - depth_value_max); // * proj_scale;

    const float tau_max = compute_tau(depth_value_max, config_.tau_min, config_.tau_max, map_.getDataConfig());
    const float three_sigma_min = compute_three_sigma(depth_value_max, config_.sigma_min, config_.sigma_max, map_.getDataConfig());

    if (z_diff_min > 1.25 * tau_max) { // behind of surface
        return se::VarianceState::Undefined;
    }
    else if (z_diff_max < -1.25 * three_sigma_min) { // guranteed free space
        return se::VarianceState::Constant;
    }
    else {
        return se::VarianceState::Gradient;
    }
}


template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
template<class SensorTDummy>
typename std::enable_if_t<std::is_same<SensorTDummy, PinholeCamera>::value, void>
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>::operator()(const Eigen::Vector3i& octant_coord,
                                                                                                          const int octant_size,
                                                                                                          const int octant_depth,
                                                                                                          se::OctantBase* octant_ptr)
{
    /// Approximate max and min depth to quickly check if the node is behind the camera or maximum depth.
    // Compute the node centre's depth in the camera frame

    /// CHANGE: USE voxelToPoint(...)
    Eigen::Vector3f node_centre_point_W;
    map_.voxelToPoint(octant_coord, octant_size, node_centre_point_W);

    const Eigen::Vector3f node_centre_point_S = (T_SW_ * node_centre_point_W.homogeneous()).head(3);

    // Extend and reduce the depth by the sphere radius covering the entire cube
    const float approx_depth_value_max = node_centre_point_S.z() + octant_size * size_to_radius_ * map_res_;
    const float approx_depth_value_min = node_centre_point_S.z() - octant_size * size_to_radius_ * map_res_;

    /// CASE 0.1 (OUT OF BOUNDS): Block is behind the camera or behind the maximum depth value
    if (approx_depth_value_min > max_depth_value_ || approx_depth_value_max < zero_depth_band_) { // TODO: Alternative sensor_.near_plane.
        return;
    }

    // Compute the 8 corners of the node to be evaluated
    /// CHANGE: USE voxelToCornerPoints(...)
    Eigen::Matrix<float, 3, 8> node_corner_points_W;
    map_.voxelToCornerPoints(octant_coord, octant_size, node_corner_points_W);
    Eigen::Matrix<float, 3, 8> node_corner_points_S = (T_SW_ * node_corner_points_W.colwise().homogeneous()).topRows(3);

    Eigen::VectorXi node_corners_infront(8);
    node_corners_infront << 1, 1, 1, 1, 1, 1, 1, 1;
    for (int corner_idx = 0; corner_idx < 8; corner_idx++) {
        if (node_corner_points_S(2, corner_idx) < zero_depth_band_) {
            node_corners_infront(corner_idx) = 0;
        }
    }

    int num_node_corners_infront = node_corners_infront.sum();

    /// CASE 0.2 (OUT OF BOUNDS): Node is behind the camera.
    if (num_node_corners_infront == 0) {
        return;
    }

    // Project the 8 corners into the image plane
    Eigen::Matrix2Xf proj_node_corner_pixels_f(2, 8);
    const Eigen::VectorXf node_corners_diff = node_corner_points_S.row(2);
    std::vector<srl::projection::ProjectionStatus> proj_node_corner_stati;
    sensor_.model.projectBatch(node_corner_points_S, &proj_node_corner_pixels_f, &proj_node_corner_stati);

    /// Approximate a 2D bounding box covering the projected node in the image plane.
    bool should_split = false;
    bool projects_inside = false;
    se::VarianceState variance_state = se::VarianceState::Gradient; ///<< -1 := low variance infront of the surface, 0 := high variance, 1 = low_variance behind the surface.
    se::Pixel pooling_pixel = se::Pixel::crossingUnknownPixel();    ///<< min, max pixel batch depth + crossing frustum state + contains unknown values state.

    if (octant_depth < octree_.getBlockDepth() + 1) {
        if (num_node_corners_infront < 8) {
            /// CASE 1 (CAMERA IN NODE):
            if (cameraInNode(octant_coord, octant_size, se::math::to_inverse_transformation(T_SW_))) {
                should_split = true;
                /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary
            }
            else if (crossesFrustum(proj_node_corner_stati)) {
                should_split = true;
                /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary without a corner reprojecting
            }
            else if (sensor_.sphereInFrustumInf(node_centre_point_S, octant_size * size_to_radius_ * map_res_)) {
                should_split = true;
            }
            else {
                return;
            }
        }
        else {
            // Compute the minimum and maximum pixel values to generate the bounding box
            const Eigen::Vector2i img_bb_min = proj_node_corner_pixels_f.rowwise().minCoeff().cast<int>();
            const Eigen::Vector2i img_bb_max = proj_node_corner_pixels_f.rowwise().maxCoeff().cast<int>();
            const float node_dist_min_m = node_corners_diff.minCoeff();
            const float node_dist_max_m = node_corners_diff.maxCoeff();

            pooling_pixel = depth_pooling_img_.conservativeQuery(img_bb_min, img_bb_max);

            /// CASE 0.3 (OUT OF BOUNDS): The node is outside frustum (i.e left, right, below, above) or
            ///                           all pixel values are unknown -> return intermediately
            if (pooling_pixel.status_known == se::Pixel::statusKnown::unknown) {
                return;
            }

            /// CASE 0.4 (OUT OF BOUNDS): The node is behind surface
            if (node_dist_min_m > pooling_pixel.max + config_.tau_max) { // TODO: Can be changed to node_dist_max_m?
                return;
            }

            variance_state = computeVariance(pooling_pixel.min, pooling_pixel.max, node_dist_min_m, node_dist_max_m);

            /// CASE 1 (REDUNDANT DATA): Depth values in the bounding box are far away from the node or unknown (1).
            ///                          The node to be evaluated is free (2) and fully observed (3),
            if (variance_state != se::VarianceState::Gradient) {
                typename OctreeType::DataType child_data = (octant_ptr->isBlock()) ? static_cast<BlockType*>(octant_ptr)->getMaxData() : static_cast<NodeType*>(octant_ptr)->getData();

                if (child_data.observed // Check if the child is fully observed (i.e. all children are observed) // TODO: incooperate MAX occupancy
                    && child_data.occupancy * child_data.weight <= 0.95 * MapType::DataType::min_occupancy) {
                    return;
                }
            }

            /// CASE 2 (FRUSTUM BOUNDARY): The node is crossing the frustum boundary
            if (pooling_pixel.status_crossing == se::Pixel::statusCrossing::crossing) {
                should_split = true;
            }

            /// CASE 3: The node is inside the frustum, but projects into partially known pixel
            else if (pooling_pixel.status_known == se::Pixel::statusKnown::part_known) {
                should_split = true;
            }

            /// CASE 4: The node is inside the frustum with only known data + node has a potential high variance
            else if (variance_state == se::VarianceState::Gradient) {
                should_split = true;
            }

            projects_inside = (pooling_pixel.status_known == se::Pixel::known);
        }
    }

    if (should_split) {
        // Returns a pointer to the according node if it has previously been allocated.
        if (octant_ptr->isBlock()) { // Evaluate the node directly if it is a voxel block
            octant_ptr->setActive(true);
            // Cast from node to voxel block
#pragma omp critical(block_lock)
            { // Add voxel block to voxel block list for later update and up-propagation
                allocation_list_.block_list.push_back(octant_ptr);
                allocation_list_.variance_state_list.push_back(variance_state);
                allocation_list_.projects_inside_list.push_back(projects_inside);
            }
        }
        else {
            NodeType* node_ptr = static_cast<NodeType*>(octant_ptr);
            const int child_size = node_ptr->getSize() / 2;
            // Split! Start recursive process
            octree_.allocateChildren(node_ptr);
#pragma omp parallel for
            for (int child_idx = 0; child_idx < 8; ++child_idx) {
                (*this)(node_ptr->getChildCoord(child_idx), child_size, octant_depth + 1, node_ptr->getChild(child_idx));
            }
        }
    }
    else {
        assert(octant_depth);

        if (octant_ptr->isBlock()) {
#pragma omp critical(block_lock)
            { // Add node to node list for later up propagation (finest node for this branch)
                allocation_list_.block_list.push_back(octant_ptr);
                allocation_list_.variance_state_list.push_back(variance_state);
                allocation_list_.projects_inside_list.push_back(projects_inside);
            }
        }
        else if (variance_state == se::VarianceState::Constant) {
#pragma omp critical(node_lock)
            { // Add node to node list for later up propagation (finest node for this branch)
                allocation_list_.node_list.push_back(octant_ptr);
            }
        } // else node has low variance behind surface (ignore)
    }
}

} // namespace se

#endif // SE_VOXEL_CARVER_IMPL_HPP
