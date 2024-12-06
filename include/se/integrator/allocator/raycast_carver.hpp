/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCAST_CARVER_HPP
#define SE_RAYCAST_CARVER_HPP

#include "se/common/math_util.hpp"
#include "se/integrator/allocator/dense_pooling_image.hpp"
#include "se/integrator/allocator/volume_carver.hpp"
#include "se/map/octree/propagator.hpp"

namespace se {
namespace fetcher {


template<typename MapT, typename SensorT>
inline std::vector<se::OctantBase*> frustum(MapT& map, const SensorT& sensor, const Eigen::Matrix4f& T_WS);


} // namespace fetcher


template<typename MapT, typename SensorT>
class RaycastCarver {
    public:
    typedef typename MapT::OctreeType OctreeType;

    /**
     * \brief The config file of the raycast carver
     *
     * \param[in] map   The map allocate the frustum in
     */
    struct RaycastCarverConfig {
        RaycastCarverConfig(const MapT& map) : truncation_boundary(map.getRes() * map.getDataConfig().truncation_boundary_factor), band(2 * truncation_boundary)
        {
        }

        const float truncation_boundary;
        const float band;
    };

    /**
     * \brief Setup the raycast carver.
     *
     * \param[in]  map                  The reference to the map to be updated.
     * \param[in]  sensor               The sensor model.
     * \param[in]  depth_img            The depth image to be integrated.
     * \param[in]  T_WS                 The transformation from sensor to world frame.
     * \param[in]  frame                The frame number to be integrated.
     */
    RaycastCarver(MapT& map, const SensorT& sensor, const se::Image<float>& depth_img, const Eigen::Matrix4f& T_WS, const int frame);

    /**
     * \brief Allocate a band around the depth measurements using a raycasting approach
     *
     * \retrun The allocated blocks
     */
    std::vector<se::OctantBase*> operator()();

    MapT& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const se::Image<float>& depth_img_;
    const Eigen::Matrix4f& T_WS_;
    const int frame_;
    const RaycastCarverConfig config_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace se

#include "impl/raycast_carver_impl.hpp"

#endif // SE_RAYCAST_CARVER_HPP
