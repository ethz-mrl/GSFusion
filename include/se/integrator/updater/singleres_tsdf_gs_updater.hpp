/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SINGLERES_TSDF_GS_UPDATER_HPP
#define SE_SINGLERES_TSDF_GS_UPDATER_HPP


#include <torch/torch.h>

#include "gs/gaussian.cuh"
#include "gs/gaussian_utils.cuh"
#include "gs/quad_tree.cuh"
#include "se/map/map.hpp"
#include "se/sensor/sensor.hpp"


namespace se {

// Single-res TSDF updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
class GSUpdater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT> {
    public:
    typedef Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;

    struct GSUpdaterConfig {
        GSUpdaterConfig(const MapType& map) : truncation_boundary(map.getRes() * map.getDataConfig().truncation_boundary_factor)
        {
        }

        const float truncation_boundary;
    };

    /**
     * \param[in]  map         The reference to the map to be updated.
     * \param[in]  sensor      The sensor model.
     * \param[in]  gs_model    The Gaussian model.
     * \param[in]  gs_cam_list The keyframe list of gs::Camera to store camera parameters.
     * \param[in]  gt_img_list The keyframe list of torch::Tensor to store color images.
     * \param[in]  data_queue  The queue to store visualization data for GUI
     * \param[in]  depth_img   The depth image to be integrated.
     * \param[in]  colour_img  The colour image to be integrated or nullptr if none.
     * \param[in]  class_img   The semantic class image to be integrated or nullptr if none.
     * \param[in]  T_WS        The transformation from sensor to world frame.
     * \param[in]  frame       The frame number to be integrated.
     */
    GSUpdater(MapType& map,
              const SensorT& sensor,
              gs::GaussianModel& gs_model,
              std::vector<gs::Camera>& gs_cam_list,
              std::vector<torch::Tensor>& gt_img_list,
              gs::DataQueue& data_queue,
              const Image<float>& depth_img,
              const Image<rgb_t>* colour_img,
              const Image<semantics_t>* class_img,
              const Eigen::Matrix4f& T_WS,
              const int frame);

    void operator()(std::vector<OctantBase*>& block_ptrs);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    void updateVoxel(DataType& data, float sdf_value);
    void updateVoxelColour(DataType& data, rgb_t colour_value);
    void updateVoxelSemantics(DataType& data, semantics_t class_id);
    void updateGSModel(std::vector<gs::Point>& positions, std::vector<gs::Color>& colors, std::vector<float>& scales);

    MapType& map_;
    const SensorT& sensor_;
    const Image<float>& depth_img_;
    const Image<rgb_t>* colour_img_;
    const Image<semantics_t>* class_img_;
    const Eigen::Matrix4f& T_WS_;
    const int frame_;
    const GSUpdaterConfig config_;

    gs::GaussianModel& gs_model_;
    std::vector<gs::Camera>& gs_cam_list_;
    std::vector<torch::Tensor>& gt_img_list_;
    gs::DataQueue& data_queue_;
    gs::DataPacket data_packet_;
    gs::Camera cur_gs_cam_;
    torch::Tensor cur_gt_img_;
    std::vector<uint8_t> color_data_;
    bool isKeyframe_ = false;

    double start_time_;
    double end_time_;
};

} // namespace se

#include "impl/singleres_tsdf_gs_updater_impl.hpp"

#endif // SE_SINGLERES_TSDF_GS_UPDATER_HPP
