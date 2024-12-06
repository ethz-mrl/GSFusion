/*
 * SPDX-FileCopyrightText: 2018-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2018 Nils Funk
 * SPDX-FileCopyrightText: 2019-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_IO_OCTOMAP_IO_HPP
#define SE_MAP_IO_OCTOMAP_IO_HPP
#ifdef SE_OCTOMAP

#    include <octomap/octomap.h>

#    include "se/map/map.hpp"

namespace se {

/** Convert an se::Map to an OctoMap octomap::OcTree.
 * For documentation on octomap::OcTree see
 * http://octomap.github.io/octomap/doc/classoctomap_1_1OcTree.html
 *
 * A function argument is used to specify how the se::Map data is converted to OctoMap data. The
 * signature of the function must be
 * ```
 * [](octomap::OcTree& octomap, const octomap::point3d& point_W, const typename MapT::DataType& data) -> void
 * ```
 * where `octomap` is the OctoMap format octree being created, `point_W` the coordinates of the
 * point to updated expressed in the world frame and `data` the data stored in the se::Map for this
 * point. The lambda function can call `octomap.setNodeValue(point_W, value);` to set a
 * log-odds value or `octomap.updateNode(point_W, value, true);` to set a boolean value.
 *
 * \note To save the OctoMap to a file, call the
 * `octomap::OcTree::writeBinary(const std::string& filename)` method on the returned OctoMap. Make
 * sure the extension of the provided filename is `.bt`.
 *
 * \note The conversion is lossy since the OctoMap only stores occupancy probabilities in log-odds
 * for each voxel.
 *
 * \param[in] map          The map to convert.
 * \param[in] convert_data The function used to convert the map data.
 * \return A pointer to an OctoMap, allocated by `new`. Wrap it in a smart pointer or manually
 *         deallocate the memory with `delete` after use.
 */
template<typename MapT, typename FunctionT>
octomap::OcTree* to_octomap(const MapT& map, FunctionT convert_data);


/** Convert an se::Map to an OctoMap. The log-odds occupancy probability of the se::Map is saved
 * directly in the OctoMap.
 *
 * \param[in] map The map to convert.
 * \return A pointer to an OctoMap, allocated by `new`. Wrap it in a smart pointer or manually
 *         deallocate the memory with `delete` after use.
 */
template<typename MapT>
octomap::OcTree* to_octomap(const MapT& map);


/** Convert an se::Map to an OctoMap. Only a binary state, occupied or free, is saved in the
 * OctoMap. Points are considered occupied if their occupancy probability is greater than 0.5, free
 * if it is smaller than 0.5 and unknown if it is exactly 0.5. Unknown points are not saved at all
 * in the OctoMap.
 *
 * \param[in] map The map to convert.
 * \return A pointer to an OctoMap, allocated by `new`. Wrap it in a smart pointer or manually
 *         deallocate the memory with `delete` after use.
 */
template<typename MapT>
octomap::OcTree* to_binary_octomap(const MapT& map);

} // namespace se

#    include "impl/octomap_io_impl.hpp"

#else
#    error supereight 2 built without OctoMap support.
#endif // SE_OCTOMAP
#endif // SE_MAP_IO_OCTOMAP_IO_HPP
