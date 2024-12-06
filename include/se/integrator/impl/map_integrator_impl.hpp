/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

#include "se/integrator/updater/updater.hpp"

namespace se {


static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord, const int octant_size)
{
    return octant_coord.cast<float>() + sample_offset_frac * octant_size;
}


namespace details {

template<Field FldT, Res ResT>
struct GSIntegrateImplD {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          gs::GaussianModel& gs_model,
                          std::vector<gs::Camera>& gs_cam_list,
                          std::vector<torch::Tensor>& gt_img_list,
                          gs::DataQueue& data_queue,
                          const Image<float>& depth_img,
                          const Image<rgb_t>* colour_img,
                          const Image<semantics_t>* class_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame);
};

template<>
struct GSIntegrateImplD<Field::TSDF, Res::Single> {
    template<typename SensorT, typename MapT>
    static void integrate(MapT& map,
                          const SensorT& sensor,
                          gs::GaussianModel& gs_model,
                          std::vector<gs::Camera>& gs_cam_list,
                          std::vector<torch::Tensor>& gt_img_list,
                          gs::DataQueue& data_queue,
                          const Image<float>& depth_img,
                          const Image<rgb_t>* colour_img,
                          const Image<semantics_t>* class_img,
                          const Eigen::Matrix4f& T_WS,
                          const unsigned int frame)
    {
        // Allocation
        TICK("allocation")
        RaycastCarver raycast_carver(map, sensor, depth_img, T_WS, frame);
        std::vector<OctantBase*> block_ptrs = raycast_carver();
        TOCK("allocation")

        // Update
        TICK("update")
        GSUpdater updater(map, sensor, gs_model, gs_cam_list, gt_img_list, data_queue, depth_img, colour_img, class_img, T_WS, frame);
        updater(block_ptrs);
        TOCK("update")
    }
};

template<typename MapT>
using GSIntegrateImpl = GSIntegrateImplD<MapT::fld_, MapT::res_>;

} // namespace details


namespace integrator {

template<typename MapT, typename SensorT>
typename std::enable_if_t<MapT::col_ == Colour::On> integrate(MapT& map,
                                                              gs::GaussianModel& gs_model,
                                                              std::vector<gs::Camera>& gs_cam_list,
                                                              std::vector<torch::Tensor>& gt_img_list,
                                                              gs::DataQueue& data_queue,
                                                              const Image<float>& depth_img,
                                                              const Image<rgb_t>& colour_img,
                                                              const SensorT& sensor,
                                                              const Eigen::Matrix4f& T_WS,
                                                              const unsigned int frame)
{
    if (depth_img.width() != colour_img.width() || depth_img.height() != colour_img.height()) {
        std::ostringstream oss;
        oss << "depth (" << depth_img.width() << "x" << depth_img.height() << ") and colour (" << colour_img.width() << "x" << colour_img.height() << ") image dimensions differ";
        throw std::invalid_argument(oss.str());
    }
    details::GSIntegrateImpl<MapT>::integrate(map, sensor, gs_model, gs_cam_list, gt_img_list, data_queue, depth_img, &colour_img, nullptr, T_WS, frame);
}

} // namespace integrator

} // namespace se

#endif // SE_MAP_INTEGRATOR_IMPL_HPP
