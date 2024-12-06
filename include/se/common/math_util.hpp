/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MATH_UTIL_HPP
#define SE_MATH_UTIL_HPP


#include <Eigen/Geometry>
#include <cmath>


namespace se {
namespace math {


template<typename T>
constexpr bool is_power_of_two(T);

template<>
constexpr bool is_power_of_two<unsigned>(const unsigned x);

constexpr int log2_const(int n);

static inline unsigned power_two_up(const float x);

template<typename T>
static inline T fracf(const T& v);

template<typename T>
static inline T floorf(const T& v);

template<typename T>
static inline T fabs(const T& v);

template<typename Scalar>
static constexpr inline Scalar sq(Scalar a);

template<typename Scalar>
static constexpr inline Scalar cu(Scalar a);

/** \brief Perform the integer division a/b and round towards positive infinity.
 * \note Not tested for negative integers.
 */
template<typename Int>
constexpr typename std::enable_if_t<std::is_integral_v<Int>, Int> div_ceil(Int a, Int b);

/** \brief Perform the division a/b and round towards positive infinity.
 */
template<typename Float>
typename std::enable_if_t<std::is_floating_point_v<Float>, Float> div_ceil(Float a, Float b);

template<typename A, typename B, typename C, typename D>
constexpr typename std::common_type_t<A, B, C, D> add_clamp(A a, B b, C low, D high);

template<typename Scalar>
static inline bool in(const Scalar v, const Scalar a, const Scalar b);

static inline Eigen::Vector3f to_translation(const Eigen::Matrix4f& T);

static inline Eigen::Matrix3f to_rotation(const Eigen::Matrix4f& T);

static inline Eigen::Matrix4f to_transformation(const Eigen::Vector3f& t);

static inline Eigen::Matrix4f to_transformation(const Eigen::Matrix3f& R);

static inline Eigen::Matrix4f to_transformation(const Eigen::Matrix3f& R, const Eigen::Vector3f& t);

static inline Eigen::Vector3f to_inverse_translation(const Eigen::Matrix4f& T);

static inline Eigen::Matrix3f to_inverse_rotation(const Eigen::Matrix4f& T);

static inline Eigen::Matrix4f to_inverse_transformation(const Eigen::Matrix4f& T);

template<typename T>
static inline typename std::enable_if<std::is_arithmetic<T>::value, T>::type clamp(const T& f, const T& a, const T& b);

static inline void clamp(Eigen::Ref<Eigen::VectorXf> res, const Eigen::Ref<const Eigen::VectorXf> a, const Eigen::Ref<Eigen::VectorXf> b);

template<typename R, typename A, typename B>
static inline void clamp(Eigen::MatrixBase<R>& res, const Eigen::MatrixBase<A>& a, const Eigen::MatrixBase<B>& b);

/*! \brief Compute the normal vector of a plane defined by 3 points.
 * The direction of the normal depends on the order of the points.
 */
static Eigen::Vector3f plane_normal(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);

/**
 * \brief hat-operator
 *
 * It takes in the 3-vector representation ``omega`` (= rotation vector) and
 * returns the corresponding matrix representation of Lie algebra element.
 *
 * Formally, the hat()-operator of SO(3) is defined as
 *
 *   ``hat(.): R^3 -> R^{3x3},  hat(omega) = sum_i omega_i * G_i``
 *   (for i=0,1,2)
 *
 * with ``G_i`` being the ith infinitesimal generator of SO(3).
 *
 * The corresponding inverse is the vee()-operator, see below.
 *
 * \param[in] omega rotation vector
 *
 * \return Corresponding matrix representation of Lie algebra element.
 */
static Eigen::Matrix3f hat(const Eigen::Vector3f& omega);

static Eigen::Matrix3f exp_and_theta(const Eigen::Vector3f& omega, float& theta);

/**
 * \brief Group exponential
 *
 * This functions takes in an element of tangent space (= twist ``a``) and
 * returns the corresponding element of the group SE(3).
 *
 * The first three components of ``a`` represent the translational part
 * ``upsilon`` in the tangent space of SE(3), while the last three components
 * of ``a`` represents the rotation vector ``omega``.
 * To be more specific, this function computes ``expmat(hat(a))`` with
 * ``expmat(.)`` being the matrix exponential and ``hat(.)`` the hat-operator
 * of SE(3), see below.
 */
static Eigen::Matrix4f exp(const Eigen::Matrix<float, 6, 1>& a);


} // namespace math
} // namespace se


#include "impl/math_util_impl.hpp"


#endif // SE_MATH_UTIL_HPP
