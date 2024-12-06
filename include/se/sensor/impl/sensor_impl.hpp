/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SENSOR_IMPL_HPP
#define SE_SENSOR_IMPL_HPP

namespace se {


template<typename DerivedT>
template<typename ConfigT>
SensorBase<DerivedT>::SensorBase(const ConfigT& c) : left_hand_frame(false), near_plane(c.near_plane), far_plane(c.far_plane), T_BS(c.T_BS)
{
}


template<typename DerivedT>
SensorBase<DerivedT>::SensorBase(const DerivedT& d) : left_hand_frame(d.left_hand_frame), near_plane(d.near_plane), far_plane(d.far_plane), T_BS(d.T_BS)
{
}


template<typename DerivedT>
template<typename ValidPredicate>
bool SensorBase<DerivedT>::getPixelValue(const Eigen::Vector2f& pixel_f, const se::Image<float>& img, float& img_value, ValidPredicate valid_predicate) const
{
    if (!this->underlying().model.isInImage(pixel_f)) {
        return false;
    }
    Eigen::Vector2i pixel = se::round_pixel(pixel_f);
    img_value = img(pixel.x(), pixel.y());
    // Return false for invalid depth measurement
    if (!valid_predicate(img_value)) {
        return false;
    }
    return true;
}


template<typename DerivedT>
int SensorBase<DerivedT>::computeIntegrationScale(const Eigen::Vector3f& block_centre_S, const float map_res, const int last_scale, const int min_scale, const int max_block_scale) const
{
    return this->underlying().computeIntegrationScaleImpl(block_centre_S, map_res, last_scale, min_scale, max_block_scale);
}


template<typename DerivedT>
float SensorBase<DerivedT>::nearDist(const Eigen::Vector3f& ray_S) const
{
    return this->underlying().nearDistImpl(ray_S);
}


template<typename DerivedT>
float SensorBase<DerivedT>::farDist(const Eigen::Vector3f& ray_S) const
{
    return this->underlying().farDistImpl(ray_S);
}


template<typename DerivedT>
float SensorBase<DerivedT>::measurementFromPoint(const Eigen::Vector3f& point_S) const
{
    return this->underlying().measurementFromPointImpl(point_S);
}


template<typename DerivedT>
bool SensorBase<DerivedT>::pointInFrustum(const Eigen::Vector3f& point_S) const
{
    return this->underlying().pointInFrustumImpl(point_S);
}


template<typename DerivedT>
bool SensorBase<DerivedT>::pointInFrustumInf(const Eigen::Vector3f& point_S) const
{
    return this->underlying().pointInFrustumInfImpl(point_S);
}


template<typename DerivedT>
bool SensorBase<DerivedT>::sphereInFrustum(const Eigen::Vector3f& centre_S, const float radius) const
{
    return this->underlying().sphereInFrustumImpl(centre_S, radius);
}


template<typename DerivedT>
bool SensorBase<DerivedT>::sphereInFrustumInf(const Eigen::Vector3f& centre_S, const float radius) const
{
    return this->underlying().sphereInFrustumInfImpl(centre_S, radius);
}


template<typename DerivedT>
std::string SensorBase<DerivedT>::type()
{
    return DerivedT::typeImpl();
}


template<typename DerivedT>
DerivedT& SensorBase<DerivedT>::underlying()
{
    return static_cast<DerivedT&>(*this);
}


template<typename DerivedT>
const DerivedT& SensorBase<DerivedT>::underlying() const
{
    return static_cast<const DerivedT&>(*this);
}


} // namespace se

#endif // SE_SENSOR_IMPL_HPP
