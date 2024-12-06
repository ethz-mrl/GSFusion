/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2018-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_IO_HPP
#define SE_OCTREE_IO_HPP

#include <fstream>
#include <iostream>
#include <sstream>

#include "se/map/octree/octree.hpp"

namespace se {
namespace io {

/**
 * \brief Generate a 3D slice of the octree.
 *
 * \tparam GetValueF
 * \param filename      The file name to save the 3D slice to
 * \param lower_coord   The lower coords of the bounding box
 * \param upper_coord   The upper coords of the bounding box
 * \param get_value     The get value function (get_value(const Eigen::Vector3i)) extracting the value
 *                      for given voxel coordinates
 *
 * \return True if the file can be read, false otherwise
 */
template<typename GetValueF>
bool save_3d_slice_vtk(const std::string& filename, const Eigen::Vector3i& lower_coord, const Eigen::Vector3i& upper_coord, GetValueF& get_value);

} // namespace io
} // namespace se


#include "impl/octree_io_impl.hpp"


#endif // SE_OCTREE_IO_HPP
