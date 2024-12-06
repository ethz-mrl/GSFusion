/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_UPDATER_HPP
#define SE_UPDATER_HPP


namespace se {

template<typename MapT, typename SensorT>
class GSUpdater {
    public:
    GSUpdater(MapT& map,
              const SensorT& sensor,
              gs::GaussianModel& gs_model,
              std::vector<gs::Camera>& gs_cam_list,
              std::vector<torch::Tensor>& gt_img_list,
              gs::DataQueue& data_queue,
              const se::Image<float>& depth_img,
              const se::Image<rgb_t>* colour_img,
              const Image<semantics_t>* class_img,
              const Eigen::Matrix4f& T_WS,
              const int frame);

    template<typename UpdateListT>
    void operator()(UpdateListT& updating_list);
};

template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class GSUpdater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single, BlockSize>, SensorT>;


} // namespace se

#include "singleres_tsdf_gs_updater.hpp"

#endif // SE_UPDATER_HPP
