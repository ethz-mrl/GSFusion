/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_PINHOLE_CAMERA_HPP
#define SE_PINHOLE_CAMERA_HPP


namespace se {

struct PinholeCameraConfig : public SensorBaseConfig {
    /** The sensor's horizontal focal length in pixels.
     */
    float fx = std::numeric_limits<float>::quiet_NaN();

    /** The sensor's vertical focal length in pixels.
     */
    float fy = std::numeric_limits<float>::quiet_NaN();

    /** The sensor's optical centre horizontal coordinate in pixels.
     */
    float cx = std::numeric_limits<float>::quiet_NaN();

    /** The sensor's optical centre vertical coordinate in pixels.
     */
    float cy = std::numeric_limits<float>::quiet_NaN();

    /** Reads the struct members from the "sensor" node of a YAML file. Members not present in the
     * YAML file aren't modified.
     */
    void readYaml(const std::string& filename);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const PinholeCameraConfig& c);


class PinholeCamera : public SensorBase<PinholeCamera> {
    public:
    PinholeCamera(const PinholeCameraConfig& config);

    int computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre, const float map_res, const int last_scale, const int min_scale, const int max_block_scale) const;

    float nearDistImpl(const Eigen::Vector3f& ray_S) const;

    float farDistImpl(const Eigen::Vector3f& ray_S) const;

    float measurementFromPointImpl(const Eigen::Vector3f& point_S) const;

    bool pointInFrustumImpl(const Eigen::Vector3f& point_S) const;

    bool pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const;

    bool sphereInFrustumImpl(const Eigen::Vector3f& centre_S, const float radius) const;

    bool sphereInFrustumInfImpl(const Eigen::Vector3f& centre_S, const float radius) const;

    static std::string typeImpl();

    srl::projection::PinholeCamera<srl::projection::NoDistortion> model;
    float scaled_pixel;

    /** \brief The horizontal field of view in radians. */
    float horizontal_fov;

    /** \brief The vertical field of view in radians. */
    float vertical_fov;

    void computeFrustumVertices();
    void computeFrustumNormals();

    static constexpr int num_frustum_vertices_ = 8;
    static constexpr int num_frustum_normals_ = 6;
    Eigen::Matrix<float, 3, num_frustum_vertices_> frustum_vertices_;
    Eigen::Matrix<float, 4, num_frustum_normals_> frustum_normals_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace se

#include "impl/pinhole_camera_impl.hpp"

#endif // SE_PINHOLE_CAMERA_HPP
