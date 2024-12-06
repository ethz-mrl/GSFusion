/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VISITOR_HPP
#define SE_VISITOR_HPP

#include <optional>

#include "octree.hpp"
#include "se/map/data.hpp"
#include "se/map/octree/allocator.hpp"
#include "se/map/octree/fetcher.hpp"

/**
 * Helper wrapper to traverse the octree. All functions take a const octree references and as no manipulation of the octree is done.
 */
namespace se {
namespace visitor {


/// Single/multi-res get data functions

/**
 * \brief Get the voxel data for a given coordinate.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The data in the voxel to be accessed
 *         Returns init data if block is not allocated
 */
template<typename OctreeT>
typename OctreeT::DataType getData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord);

/**
 * \brief Get the voxel data for a given coordinate.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct block.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] block_ptr   The pointer to the block to checked first
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The data in the voxel to be accessed
 *         Returns init data if block is not allocated
 */
template<typename OctreeT, typename BlockT>
typename OctreeT::DataType getData(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord);


/// Multi-res get data functions

/**
 * \brief Get the voxel data for a given coordinate and desired scale.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree_ptr     The pointer to the octree
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the data is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The data in octant at the returned scale
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, typename OctreeT::DataType> getData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);

/**
 * \brief Get the voxel data for a given coordinate and desired scale.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct block.
 *        The function returns init data if the data is not allocated.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] block_ptr      The pointer to the block to checked first
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the data is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The data in octant at the returned scale
 */
template<typename OctreeT, typename BlockT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, typename OctreeT::DataType>
getData(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);

/**
 * \brief Get the max occupancy data at a given scale.
 *
 * \tparam OctreeT      The type of octree used (has to be of field type occupancy and multi-res)
 * \param octree        The reference to the octree
 * \param voxel_coord   The voxel coordinates in [voxel] to be accessed
 * \param scale_desired The scale to be accessed
 * \return The max data at the requested scale.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::DataType::fld_ == Field::Occupancy, typename OctreeT::DataType> getMaxData(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired);

// TODO: Reduce getField functions for single and multi-res to one

/// Single/Multi-res get field functions

/**
 * \brief Get the field value for a given coordinate.
 *        The function returns {}/invalid if the data is invalid.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The field value to be accessed if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT>
std::optional<float> getField(const OctreeT& octree, const Eigen::Vector3i& voxel_coord);

/**
 * \brief Get the field value for a given coordinate.
 *        The function returns {}/invalid if the data is invalid.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct octant.
 *
 * \tparam OctreeT        The type of the octree used
 * \param[in] octree      The reference to the octree
 * \param[in] block_ptr   The pointer to the block to checked first
 * \param[in] voxel_coord The voxel coordinates to be accessed
 *
 * \return The field value to be accessed if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT, typename BlockT>
std::optional<float> getField(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord);


/// Multi-res get field functions

/**
 * \brief Get the field value for a given coordinate and desired scale.
 *        The function returns {}/invalid if the data is invalid.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the field value is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The field value at the returned scale if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<float>> getField(const OctreeT& octree, const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);

/**
 * \brief Get the field value for a given coordinate and desired scale.
 *        The function returns {}/invalid if the data is invalid.
 *        The function checks first if the voxel coordinates are contained in the provided block pointer.
 *        If this is not the case the function fetches the correct octant.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] block_ptr      The pointer to the block to checked first
 * \param[in] voxel_coord    The voxel coordinates to be accessed
 * \param[in] scale_desired  The scale to fetch the data from (init data for MultiresTSDF at node level)
 * \param[in] scale_returned The scale the field value is returned from (max (scale desired, finest scale with valid data)
 *
 * \return The field value at the returned scale if the data is valid, {}/invalid otherwise
 */
template<typename OctreeT, typename BlockT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<float>>
getField(const OctreeT& octree, BlockT* block_ptr, const Eigen::Vector3i& voxel_coord, const int scale_desired, int& scale_returned);


/** \brief Interpolate the field value at the supplied coordinates.
 *
 * \tparam    OctreeT       The type of the octree used.
 * \param[in] octree        The single-resolution octree containing the field data.
 * \param[in] voxel_coord_f The voxel coordinates the field will be interpolated at. The coordinates
 *                          may have a fractional part.
 * \return The interpolated field value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Single, std::optional<float>> getFieldInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f);

/** \brief Interpolate the field value at the supplied coordinates. The value is interpolated at the
 * finest scale data is available at.
 *
 * \tparam    OctreeT       The type of the octree used.
 * \param[in] octree        The multi-resolution octree containing the field data.
 * \param[in] voxel_coord_f The voxel coordinates the field will be interpolated at. The coordinates
 *                          may have a fractional part.
 * \return The interpolated field value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<float>> getFieldInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f);

/** \brief Interpolate the field value at the supplied coordinates. The value is interpolated at the
 * finest scale data is available at.
 *
 * \tparam     OctreeT        The type of the octree used.
 * \param[in]  octree         The multi-resolution octree containing the field data.
 * \param[in]  voxel_coord_f  The voxel coordinates the field will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[out] scale_returned The scale the field value was interpolated at. Only valid if the
 *                            interpolation succeeded.
 * \return The interpolated field value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<float>> getFieldInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, int& scale_returned);

/** \brief Interpolate the field value at the supplied coordinates while limiting the finest scale
 * at which interpolation is performed.
 *
 * \tparam     OctreeT        The type of the octree used.
 * \param[in]  octree         The multi-resolution TSDF octree containing the field data.
 * \param[in]  voxel_coord_f  The voxel coordinates the field will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[in]  scale_desired  The finest scale to interpolate the data at.
 * \param[out] scale_returned The scale the field value was interpolated at. Only valid if the
 *                            interpolation succeeded.
 * \return The interpolated field value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT>
typename std::enable_if_t<(OctreeT::fld_ == Field::TSDF && OctreeT::res_ == Res::Multi), std::optional<float>>
getFieldInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, const int scale_desired, int& scale_returned);

/** \brief Interpolate the field value at the supplied coordinates while limiting the finest scale
 * at which interpolation is performed.
 *
 * \tparam     OctreeT        The type of the octree used.
 * \param[in]  octree         The multi-resolution occupancy octree containing the field data.
 * \param[in]  voxel_coord_f  The voxel coordinates the field will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[in]  scale_desired  The finest scale to interpolate the data at.
 * \param[out] scale_returned The scale the field value was interpolated at. Only valid if the
 *                            interpolation succeeded.
 * \return The interpolated field value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::fld_ == Field::Occupancy && OctreeT::res_ == Res::Multi, std::optional<float>>
getFieldInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, const int scale_desired, int& scale_returned);


/** \brief Interpolate a value at the supplied coordinates.
 *
 * \tparam    OctreeT       The type of the octree used.
 * \tparam    GetF          The type of the function returning the value to be interpolated.
 * \param[in] octree        The single-resolution octree containing the field data.
 * \param[in] voxel_coord_f The voxel coordinates the field will be interpolated at. The coordinates
 *                          may have a fractional part.
 * \param[in] get_value     A function returning the value of type `T` to be interpolated with the
 *                          following prototype:
 *                          \code{.cpp}
 *                          template<typename OctreeT>
 *                          T get_value(const typename OctreeT::DataType& data);
 *                          \endcode
 * \return The interpolated value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT, typename GetF>
typename std::enable_if_t<OctreeT::res_ == Res::Single, std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, GetF get_value);

/** \brief Interpolate the field value at the supplied coordinates. The value is interpolated at the
 * finest scale data is available at.
 *
 * \tparam    OctreeT       The type of the octree used.
 * \tparam    GetF          The type of the function returning the value to be interpolated.
 * \param[in] octree        The multi-resolution octree containing the field data.
 * \param[in] voxel_coord_f The voxel coordinates the field will be interpolated at. The coordinates
 *                          may have a fractional part.
 * \param[in] get_value     A function returning the value of type `T` to be interpolated with the
 *                          following prototype:
 *                          \code{.cpp}
 *                          template<typename OctreeT>
 *                          T get_value(const typename OctreeT::DataType& data);
 *                          \endcode
 * \return The interpolated value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT, typename GetF>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, GetF get_value);

/** \brief Interpolate the field value at the supplied coordinates. The value is interpolated at the
 * finest scale data is available at.
 *
 * \tparam     OctreeT        The type of the octree used.
 * \tparam     GetF           The type of the function returning the value to be interpolated.
 * \param[in]  octree         The multi-resolution octree containing the field data.
 * \param[in]  voxel_coord_f  The voxel coordinates the field will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[out] scale_returned The scale the field value was interpolated at. Only valid if the
 *                            interpolation succeeded.
 * \param[in] get_value       A function returning the value of type `T` to be interpolated with the
 *                            following prototype:
 *                            \code{.cpp}
 *                            template<typename OctreeT>
 *                            T get_value(const typename OctreeT::DataType& data);
 *                            \endcode
 * \return The interpolated value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT, typename GetF>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, int& scale_returned, GetF get_value);

/** \brief Interpolate the field value at the supplied coordinates while limiting the finest scale
 * at which interpolation is performed.
 *
 * \tparam     OctreeT        The type of the octree used.
 * \tparam     GetF           The type of the function returning the value to be interpolated.
 * \param[in]  octree         The multi-resolution TSDF octree containing the field data.
 * \param[in]  voxel_coord_f  The voxel coordinates the field will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[in]  scale_desired  The finest scale to interpolate the data at.
 * \param[out] scale_returned The scale the field value was interpolated at. Only valid if the
 *                            interpolation succeeded.
 * \param[in] get_value       A function returning the value of type `T` to be interpolated with the
 *                            following prototype:
 *                            \code{.cpp}
 *                            template<typename OctreeT>
 *                            T get_value(const typename OctreeT::DataType& data);
 *                            \endcode
 * \return The interpolated value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT, typename GetF>
typename std::enable_if_t<(OctreeT::fld_ == Field::TSDF && OctreeT::res_ == Res::Multi), std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, const int scale_desired, int& scale_returned, GetF get_value);

/** \brief Interpolate the field value at the supplied coordinates while limiting the finest scale
 * at which interpolation is performed.
 *
 * \tparam     OctreeT        The type of the octree used.
 * \tparam     GetF           The type of the function returning the value to be interpolated.
 * \param[in]  octree         The multi-resolution occupancy octree containing the field data.
 * \param[in]  voxel_coord_f  The voxel coordinates the field will be interpolated at. The
 *                            coordinates may have a fractional part.
 * \param[in]  scale_desired  The finest scale to interpolate the data at.
 * \param[out] scale_returned The scale the field value was interpolated at. Only valid if the
 *                            interpolation succeeded.
 * \param[in] get_value       A function returning the value of type `T` to be interpolated with the
 *                            following prototype:
 *                            \code{.cpp}
 *                            template<typename OctreeT>
 *                            T get_value(const typename OctreeT::DataType& data);
 *                            \endcode
 * \return The interpolated value if the data is valid, std::nullopt otherwise.
 */
template<typename OctreeT, typename GetF>
typename std::enable_if_t<OctreeT::fld_ == Field::Occupancy && OctreeT::res_ == Res::Multi, std::optional<std::invoke_result_t<GetF, typename OctreeT::DataType>>>
getInterp(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, const int scale_desired, int& scale_returned, GetF get_value);


/// Single-res get gradient functions

/**
 * \brief Get the field gradient for a given coordinate [float voxel coordinates].
 *        The function returns {}/invalid if the gradient is invalid.
 *
 * \tparam OctreeT          The type of the octree used
 * \param[in] octree        The reference to the octree
 * \param[in] voxel_coord_f The voxel coordinates to be accessed [float voxel coordiantes]
 *
 * \return The field gradient if the gradient is valid, {}/invalid otherwise
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Single, std::optional<Eigen::Vector3f>> getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f);


/// Multi-res get gradient functions

/**
 * \brief Get the field gradient for a given coordinate [float voxel coordinates].
 *        The function returns {}/invalid if the gradient is invalid.
 *
 * \tparam OctreeT          The type of the octree used
 * \param[in] octree        The reference to the octree
 * \param[in] voxel_coord_f The voxel coordinates to be accessed [float voxel coordiantes]
 *
 * \return The field gradient if the gradient is valid, {}/invalid otherwise
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<Eigen::Vector3f>> getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f);

/**
 * \brief Get the field gradient for a given coordinate [float voxel coordinates].
 *        The function returns {}/invalid if the gradient is invalid.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] voxel_coord_f  The voxel coordinates to be accessed [float voxel coordiantes]
 * \param[in] scale_returned The scale the gradient has been computed at
 *
 * \return The field gradient if the gradient is valid, {}/invalid otherwise
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<Eigen::Vector3f>> getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, int& scale_returned);

/**
 * \brief Get the field gradient for a given coordinate [float voxel coordinates] and desired scale.
 *        The function returns {}/invalid if the gradient is invalid.
 *
 * \tparam OctreeT           The type of the octree used
 * \param[in] octree         The reference to the octree
 * \param[in] voxel_coord_f  The voxel coordinates to be accessed [float voxel coordiantes]
 * \param[in] scale_desired  The finest scale to compute the gradient at
 * \param[in] scale_returned The scale the gradient has been computed at (max (scale desired, finest common neighbour scale)
 *
 * \return The field gradient if the gradient is valid, {}/invalid otherwise
 */
template<typename OctreeT>
typename std::enable_if_t<OctreeT::res_ == Res::Multi, std::optional<Eigen::Vector3f>>
getFieldGrad(const OctreeT& octree, const Eigen::Vector3f& voxel_coord_f, const int scale_desired, int& scale_returned);

} // namespace visitor
} // namespace se

#include "impl/visitor_impl.hpp"

#endif // SE_VISITOR_HPP
