/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2018-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_IO_IMPL_HPP
#define SE_OCTREE_IO_IMPL_HPP


namespace se {
namespace io {


template<typename GetValueF>
bool save_3d_slice_vtk(const std::string& filename, const Eigen::Vector3i& lower_coord, const Eigen::Vector3i& upper_coord, GetValueF& get_value)
{
    // Open the file for writing.
    std::ofstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Unable to write file " << filename << "\n";
        return false;
    }

    std::stringstream ss_x_coord, ss_y_coord, ss_z_coord, ss_scalars;

    const int stride = 1;
    const int dimX = std::max(1, (upper_coord.x() - lower_coord.x()) / stride);
    const int dimY = std::max(1, (upper_coord.y() - lower_coord.y()) / stride);
    const int dimZ = std::max(1, (upper_coord.z() - lower_coord.z()) / stride);

    file << "# vtk DataFile Version 1.0" << std::endl;
    file << "vtk mesh generated from KFusion" << std::endl;
    file << "ASCII" << std::endl;
    file << "DATASET RECTILINEAR_GRID" << std::endl;
    file << "DIMENSIONS " << dimX << " " << dimY << " " << dimZ << std::endl;

    for (int x = lower_coord.x(); x < upper_coord.x(); x += stride) {
        ss_x_coord << x << " ";
    }
    for (int y = lower_coord.y(); y < upper_coord.y(); y += stride) {
        ss_y_coord << y << " ";
    }
    for (int z = lower_coord.z(); z < upper_coord.z(); z += stride) {
        ss_z_coord << z << " ";
    }

    for (int z = lower_coord.z(); z < upper_coord.z(); z += stride) {
        for (int y = lower_coord.y(); y < upper_coord.y(); y += stride) {
            for (int x = lower_coord.x(); x < upper_coord.x(); x += stride) {
                const auto value = get_value(Eigen::Vector3i(x, y, z));
                ss_scalars << value << std::endl;
            } // x
        }     // y
    }         // z

    file << "X_COORDINATES " << dimX << " int " << std::endl;
    file << ss_x_coord.str() << std::endl;

    file << "Y_COORDINATES " << dimY << " int " << std::endl;
    file << ss_y_coord.str() << std::endl;

    file << "Z_COORDINATES " << dimZ << " int " << std::endl;
    file << ss_z_coord.str() << std::endl;

    file << "POINT_DATA " << dimX * dimY * dimZ << std::endl;
    file << "SCALARS scalars float 1" << std::endl;
    file << "LOOKUP_TABLE default" << std::endl;
    file << ss_scalars.str() << std::endl;

    file.close();
    return true;
}


} // namespace io
} // namespace se

#endif // SE_OCTREE_IO_IMPL_HPP
