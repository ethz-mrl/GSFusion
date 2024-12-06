/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SENSOR_HPP
#define SE_SENSOR_HPP

#include <limits>

#include "se/common/image_utils.hpp"
#include "se/common/math_util.hpp"
#include "se/common/projection.hpp"
#include "se/common/yaml.hpp"
#include "se/image/image.hpp"


namespace se {

struct SensorBaseConfig {
    /** The width of images produced by the sensor in pixels.
     */
    int width = 0;

    /** The height of images produced by the sensor in pixels.
     */
    int height = 0;

    /** The sensor's near plane in metres. Avoid setting to 0 since numerical issues may arise.
     */
    float near_plane = 0.01f;

    /** The sensor's far plane in metres. Avoid setting to infinity since performance may degrade
     * significantly, for example with depth images containing really large erroneous measurements.
     */
    float far_plane = 10.0f;

    /** The transformation from the sensor frame S to the body frame B.
     */
    Eigen::Matrix4f T_BS = Eigen::Matrix4f::Identity();

    /** Reads the struct members from the "sensor" node of a YAML file. Members not present in the
     * YAML file aren't modified.
     */
    void readYaml(const std::string& filename);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


template<typename DerivedT>
class SensorBase {
    public:
    template<typename ConfigT>
    SensorBase(const ConfigT& c);

    SensorBase(const DerivedT& d);

    /**
     * \brief Get the image value for a given pixel coordindate.
     *
     * \tparam ValidPredicate
     * \param[in]  pixel_f          The pixel coordinates to get the image value from
     * \param[in]  img              The image to get the value from
     * \param[out] img_value        The image value corresponding the the pixel
     * \param[in]  valid_predicate  The lambda function verifying if the value is valid (e.g. infront of far dist)
     *
     * \return True if the image value is valid, false otherwise
     */
    template<typename ValidPredicate>
    bool getPixelValue(const Eigen::Vector2f& pixel_f, const se::Image<float>& img, float& img_value, ValidPredicate valid_predicate) const;
    /**
     * \brief Computes the scale corresponding to the back-projected pixel size
     * in voxel space.
     *
     * \param[in] block_centre_S  The coordinates of the block
     *                            centre in the sensor frame.
     * \param[in] map_res         The resolution of the map in [meter].
     * \param[in] last_scale      Scale from which propagate up voxel
     *                            values.
     * \param[in] min_scale       Finest scale at which data has been
     *                            integrated into the voxel block (-1 if no
     *                            data has been integrated yet).
     * \param[in] max_block_scale The maximum allowed scale within a
     *                            VoxelBlock.
     * \return The scale that should be used for the integration.
     */
    int computeIntegrationScale(const Eigen::Vector3f& block_centre_S, const float map_res, const int last_scale, const int min_scale, const int max_block_scale) const;

    /**
     * \brief Return the minimum distance at which measurements are available
     * along the ray passing through pixels x and y.
     *
     * This differs from the PinholeCamera::near_plane since the near_plane is
     * a z-value while nearDist is a distance along a ray.
     *
     * \param[in] ray_S The ray starting from the sensor centre and expressed
     *                  in the sensor frame along which nearDist
     *                  will be computed.
     * \return The minimum distance along the ray through the pixel at which
     *         valid measurements may be encountered.
     */
    float nearDist(const Eigen::Vector3f& ray_S) const;

    /**
     * \brief Return the maximum distance at which measurements are available
     * along the ray passing through pixels x and y.
     *
     * This differs from the PinholeCamera::far_plane since the far_plane is a
     * z-value while farDist is a distance along a ray.
     *
     * \param[in] ray_S The ray starting from the sensor centre and expressed
     *                  in the sensor frame along which nearDist
     *                  will be computed.
     * \return The maximum distance along the ray through the pixel at which
     *         valid measurements may be encountered.
     */
    float farDist(const Eigen::Vector3f& ray_S) const;

    /**
     * \brief Convert a point in the sensor frame into a depth measurement.
     * For the PinholeCamera this means returning the z-coordinate
     * of the point.
     *
     * \param[in] point_C A point observed by the sensor expressed in the
     *                    sensor frame.
     * \return The depth value that the sensor would get from this point.
     */
    float measurementFromPoint(const Eigen::Vector3f& point_S) const;

    /**
     * \brief Test whether a 3D point in sensor coordinates is inside the
     * sensor frustum.
     */
    bool pointInFrustum(const Eigen::Vector3f& point_S) const;

    /**
     * \brief Test whether a 3D point in sensor coordinates is inside the
     * sensor frustum.
     *
     * The difference from PinholeCamera::pointInFrustum is that it is assumed
     * that the far plane is at infinity.
     */
    bool pointInFrustumInf(const Eigen::Vector3f& point_S) const;

    /**
     * \brief Test whether a sphere in sensor coordinates is at least partially inside the sensor
     * frustum.
     *
     * It is tested whether the sphere's centre is inside the sensor frustum
     * offest outwards by the sphere's radius. This is a quick test that in
     * some rare cases may return a sphere as being visible although it isn't.
     */
    bool sphereInFrustum(const Eigen::Vector3f& centre_S, const float radius) const;

    /**
     * \brief Test whether a sphere in sensor coordinates is at least partially inside the sensor
     * frustum.
     *
     * The difference from PinholeCamera::sphereInFrustum is that it is assumed
     * that the far plane is at infinity.
     */
    bool sphereInFrustumInf(const Eigen::Vector3f& centre_S, const float radius) const;

    /**
     * \brief Return the sensor type as a string.
     */
    static std::string type();

    bool left_hand_frame;
    float near_plane;
    float far_plane;
    Eigen::Matrix4f T_BS;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    // Make sure the derived class and the template parameter are the same (i.e. prevent class D1 : Base<D2>)
    SensorBase(){};
    friend DerivedT;

    // Simplify access to derived member functions
    DerivedT& underlying();
    const DerivedT& underlying() const;
};


} // namespace se

#include "impl/sensor_impl.hpp"
#include "pinhole_camera.hpp"

#endif // SE_SENSOR_HPP
