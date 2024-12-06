/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/str_utils.hpp"
#include "se/sensor/sensor.hpp"


void se::PinholeCameraConfig::readYaml(const std::string& filename)
{
    // Read the base class members.
    SensorBaseConfig::readYaml(filename);

    // Open the file for reading.
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

    // Get the node containing the sensor configuration.
    const cv::FileNode node = fs["sensor"];

    // Read the config parameters.
    se::yaml::subnode_as_float(node, "fx", fx);
    se::yaml::subnode_as_float(node, "fy", fy);
    se::yaml::subnode_as_float(node, "cx", cx);
    se::yaml::subnode_as_float(node, "cy", cy);
}


std::ostream& se::operator<<(std::ostream& os, const se::PinholeCameraConfig& c)
{
    os << str_utils::value_to_pretty_str(c.width, "width") << " px\n";
    os << str_utils::value_to_pretty_str(c.height, "height") << " px\n";
    os << str_utils::value_to_pretty_str(c.near_plane, "near_plane") << " m\n";
    os << str_utils::value_to_pretty_str(c.far_plane, "far_plane") << " m\n";
    os << str_utils::value_to_pretty_str(c.fx, "fx") << " px\n";
    os << str_utils::value_to_pretty_str(c.fy, "fy") << " px\n";
    os << str_utils::value_to_pretty_str(c.cx, "cx") << " px\n";
    os << str_utils::value_to_pretty_str(c.cy, "cy") << " px\n";
    os << str_utils::eigen_matrix_to_pretty_str(c.T_BS, "T_BS") << "\n";
    return os;
}


// Explicit template class instantiation
template class srl::projection::PinholeCamera<srl::projection::NoDistortion>;

// Used for initializing a PinholeCamera.
const srl::projection::NoDistortion _distortion;

// Static variables
constexpr int se::PinholeCamera::num_frustum_vertices_;
constexpr int se::PinholeCamera::num_frustum_normals_;


se::PinholeCamera::PinholeCamera(const PinholeCameraConfig& c) : se::SensorBase<se::PinholeCamera>(c), model(c.width, c.height, c.fx, c.fy, c.cx, c.cy, _distortion), scaled_pixel(1 / c.fx)
{
    left_hand_frame = (c.fx < 0.0f) ^ (c.fy < 0.0f);

    computeFrustumVertices();
    computeFrustumNormals();

    assert(c.width > 0);
    assert(c.height > 0);
    assert(c.near_plane > 0.0f);
    assert(c.far_plane > c.near_plane);
    assert(!std::isnan(c.fx));
    assert(!std::isnan(c.fy));
    assert(!std::isnan(c.cx));
    assert(!std::isnan(c.cy));

    horizontal_fov = 2.0f * atanf(c.width / (2.0f * c.fx));
    vertical_fov = 2.0f * atanf(c.height / (2.0f * c.fy));
}


int se::PinholeCamera::computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre_S, const float map_res, const int last_scale, const int min_scale, const int max_block_scale) const
{
    const float dist = block_centre_S.z();
    // Compute the side length in metres of a pixel projected dist metres from the camera
    const float pixel_dim = dist * scaled_pixel;
    const float pv_ratio = pixel_dim / map_res;
    int scale = 0;
    if (pv_ratio < 1.5) {
        scale = 0;
    }
    else if (pv_ratio < 3) {
        scale = 1;
    }
    else if (pv_ratio < 6) {
        scale = 2;
    }
    else {
        scale = 3;
    }
    scale = std::min(scale, max_block_scale);

    Eigen::Vector3f block_centre_S_hyst = block_centre_S;
    bool recompute = false;
    if (scale > last_scale && min_scale != -1) {
        block_centre_S_hyst.z() -= 0.25;
        recompute = true;
    }
    else if (scale < last_scale && min_scale != -1) {
        block_centre_S_hyst.z() += 0.25;
        recompute = true;
    }

    if (recompute) {
        return computeIntegrationScale(block_centre_S_hyst, map_res, last_scale, -1, max_block_scale);
    }
    else {
        return scale;
    }
}


bool se::PinholeCamera::pointInFrustumImpl(const Eigen::Vector3f& point_S) const
{
    for (size_t i = 0; i < num_frustum_normals_; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = point_S.homogeneous().dot(frustum_normals_.col(i));
        if (distance < 0.0f) {
            // A negative distance means that the point is located on the opposite
            // halfspace than the one the plane normal is pointing towards
            return false;
        }
    }
    return true;
}


bool se::PinholeCamera::pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const
{
    // Skip the far plane normal
    for (size_t i = 0; i < num_frustum_normals_ - 1; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = point_S.homogeneous().dot(frustum_normals_.col(i));
        if (distance < 0.0f) {
            // A negative distance means that the point is located on the opposite
            // halfspace than the one the plane normal is pointing towards
            return false;
        }
    }
    return true;
}


bool se::PinholeCamera::sphereInFrustumImpl(const Eigen::Vector3f& center_S, const float radius) const
{
    for (size_t i = 0; i < num_frustum_normals_; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = center_S.homogeneous().dot(frustum_normals_.col(i));
        if (distance < -radius) {
            // Instead of testing for negative distance as in
            // se::PinholeCamera::pointInFrustum, test for distance smaller than
            // -radius so that the test is essentially performed on the plane offset
            // by radius.
            return false;
        }
    }
    return true;
}


bool se::PinholeCamera::sphereInFrustumInfImpl(const Eigen::Vector3f& center_S, const float radius) const
{
    // Skip the far plane normal
    for (size_t i = 0; i < num_frustum_normals_ - 1; ++i) {
        // Compute the signed distance between the point and the plane
        const float distance = center_S.homogeneous().dot(frustum_normals_.col(i));
        if (distance < -radius) {
            // Instead of testing for negative distance as in
            // se::PinholeCamera::pointInFrustum, test for distance smaller than
            // -radius so that the test is essentially performed on the plane offset
            // by radius.
            return false;
        }
    }
    return true;
}


void se::PinholeCamera::computeFrustumVertices()
{
    Eigen::Vector3f point_S;
    // Back-project the frame corners to get the frustum vertices
    // Top left
    model.backProject(Eigen::Vector2f(0.0f, 0.0f), &point_S);
    frustum_vertices_.col(0) = point_S;
    frustum_vertices_.col(4) = point_S;
    // Top right
    model.backProject(Eigen::Vector2f(model.imageWidth(), 0.0f), &point_S);
    frustum_vertices_.col(1) = point_S;
    frustum_vertices_.col(5) = point_S;
    // Bottom right
    model.backProject(Eigen::Vector2f(model.imageWidth(), model.imageHeight()), &point_S);
    frustum_vertices_.col(2) = point_S;
    frustum_vertices_.col(6) = point_S;
    // Bottom left
    model.backProject(Eigen::Vector2f(0.0f, model.imageHeight()), &point_S);
    frustum_vertices_.col(3) = point_S;
    frustum_vertices_.col(7) = point_S;
    // Scale the frustum vertices with the appropriate depth for near and far
    // plane vertices
    for (int i = 0; i < num_frustum_vertices_ / 2; ++i) {
        frustum_vertices_.col(i) *= near_plane;
        frustum_vertices_.col(num_frustum_vertices_ / 2 + i) *= far_plane;
    }
}


void se::PinholeCamera::computeFrustumNormals()
{
    // The w vector component corresponds to the distance of the plane from the
    // origin. It should be 0 for all planes other than the near and far planes.
    // Left plane vector.
    frustum_normals_.col(0).head<3>() = se::math::plane_normal(frustum_vertices_.col(4), frustum_vertices_.col(0), frustum_vertices_.col(3));
    frustum_normals_.col(0).w() = 0.0f;
    // Right plane vector.
    frustum_normals_.col(1).head<3>() = se::math::plane_normal(frustum_vertices_.col(1), frustum_vertices_.col(5), frustum_vertices_.col(6));
    frustum_normals_.col(1).w() = 0.0f;
    // Bottom plane vector.
    frustum_normals_.col(2).head<3>() = se::math::plane_normal(frustum_vertices_.col(7), frustum_vertices_.col(3), frustum_vertices_.col(2));
    frustum_normals_.col(2).w() = 0.0f;
    // Top plane vector.
    frustum_normals_.col(3).head<3>() = se::math::plane_normal(frustum_vertices_.col(5), frustum_vertices_.col(1), frustum_vertices_.col(0));
    frustum_normals_.col(3).w() = 0.0f;
    // Near plane vector.
    frustum_normals_.col(4) = Eigen::Vector4f(0.f, 0.f, 1.f, -near_plane);
    // Far plane vector.
    frustum_normals_.col(5) = Eigen::Vector4f(0.f, 0.f, -1.f, far_plane);
    // The directions of all normal vectors must be multiplied with sign(fx * fy) to account for
    // negative focal lengths. The near and far plane normals are already set with the correct
    // direction.
    float sign = model.focalLengthU() * model.focalLengthV();
    sign = sign / fabsf(sign);
    for (int i = 0; i < num_frustum_normals_ - 2; i++) {
        frustum_normals_.col(i).head<3>() *= sign;
    }
}
