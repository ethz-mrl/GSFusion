/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2023 Nils Funk
 * SPDX-FileCopyrightText: 2020-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_RAYCASTER_HPP
#define SE_RAYCASTER_HPP

#define SE_INVALID -2

#include <optional>

#include "se/common/colour_utils.hpp"
#include "se/image/image.hpp"
#include "se/map/octree/visitor.hpp"
#include "se/map/octree/voxel_block_ray_iterator.hpp"


namespace se {
namespace raycaster {

void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals, const se::Image<Eigen::Vector3f>& point_cloud, const bool is_lhc = false);

template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::Occupancy, std::optional<Eigen::Vector4f>> raycast(MapT& map,
                                                                                                             const typename MapT::OctreeType& octree,
                                                                                                             const Eigen::Vector3f& ray_origin_W,
                                                                                                             const Eigen::Vector3f& ray_dir_W,
                                                                                                             const float t_near,
                                                                                                             const float t_far,
                                                                                                             const float mu,
                                                                                                             const float step,
                                                                                                             const float largestep);

template<typename MapT>
inline typename std::enable_if_t<MapT::fld_ == se::Field::TSDF, std::optional<Eigen::Vector4f>> raycast(MapT& map,
                                                                                                        const typename MapT::OctreeType& octree,
                                                                                                        const Eigen::Vector3f& ray_origin_W,
                                                                                                        const Eigen::Vector3f& ray_dir_W,
                                                                                                        const float t_near,
                                                                                                        const float t_far,
                                                                                                        const float mu,
                                                                                                        const float step,
                                                                                                        const float largestep);

template<typename MapT, typename SensorT>
void raycast_volume(const MapT& map,
                    se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                    se::Image<Eigen::Vector3f>& surface_normals_W,
                    se::Image<int8_t>& surface_scale,
                    const Eigen::Matrix4f& T_WS,
                    const SensorT& sensor);

template<typename MapT, typename SensorT>
typename std::enable_if_t<MapT::col_ == Colour::On> raycast_volume(const MapT& map,
                                                                   se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                                                                   se::Image<Eigen::Vector3f>& surface_normals_W,
                                                                   se::Image<int8_t>& surface_scale,
                                                                   se::Image<rgb_t>& surface_colour,
                                                                   const Eigen::Matrix4f& T_WS,
                                                                   const SensorT& sensor);

template<typename MapT, typename SensorT>
typename std::enable_if_t<MapT::sem_ != Semantics::Off> raycast_volume(const MapT& map,
                                                                       se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                                                                       se::Image<Eigen::Vector3f>& surface_normals_W,
                                                                       se::Image<int8_t>& surface_scale,
                                                                       se::Image<semantics_t>& surface_class_id,
                                                                       const Eigen::Matrix4f& T_WS,
                                                                       const SensorT& sensor);

template<typename MapT, typename SensorT>
typename std::enable_if_t<MapT::col_ == Colour::On && MapT::sem_ != Semantics::Off> raycast_volume(const MapT& map,
                                                                                                   se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                                                                                                   se::Image<Eigen::Vector3f>& surface_normals_W,
                                                                                                   se::Image<int8_t>& surface_scale,
                                                                                                   se::Image<rgb_t>& surface_colour,
                                                                                                   se::Image<semantics_t>& surface_class_id,
                                                                                                   const Eigen::Matrix4f& T_WS,
                                                                                                   const SensorT& sensor);

void render_volume(uint32_t* volume_RGBA_image_data,
                   const Eigen::Vector2i& volume_RGBA_image_res,
                   const se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                   const se::Image<Eigen::Vector3f>& surface_normals_W,
                   const Eigen::Vector3f& light_W = Eigen::Vector3f::Ones(),
                   const Eigen::Vector3f& ambient_W = Eigen::Vector3f::Constant(0.1));

void render_volume_scale(uint32_t* volume_RGBA_image_data,
                         const Eigen::Vector2i& volume_RGBA_image_res,
                         const se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                         const se::Image<Eigen::Vector3f>& surface_normals_W,
                         const se::Image<int8_t>& surface_scale,
                         const Eigen::Vector3f& light_W = Eigen::Vector3f::Ones(),
                         const Eigen::Vector3f& ambient_W = Eigen::Vector3f::Constant(0.1));

void render_volume_colour(uint32_t* volume_RGBA_image_data,
                          const Eigen::Vector2i& volume_RGBA_image_res,
                          const se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                          const se::Image<Eigen::Vector3f>& surface_normals_W,
                          const se::Image<rgb_t>& surface_colour,
                          const Eigen::Vector3f& light_W = Eigen::Vector3f::Ones(),
                          const Eigen::Vector3f& ambient_W = Eigen::Vector3f::Constant(0.1));

void render_volume_class_id(uint32_t* volume_RGBA_image_data,
                            const Eigen::Vector2i& volume_RGBA_image_res,
                            const se::Image<Eigen::Vector3f>& surface_point_cloud_W,
                            const se::Image<Eigen::Vector3f>& surface_normals_W,
                            const se::Image<semantics_t>& surface_class_id,
                            const Eigen::Vector3f& light_W = Eigen::Vector3f::Ones(),
                            const Eigen::Vector3f& ambient_W = Eigen::Vector3f::Constant(0.1));

} // namespace raycaster
} // namespace se

#include "impl/raycaster_impl.hpp"

#endif // SE_RAYCASTER_HPP
