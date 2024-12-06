/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_POINT_CLOUD_IO_IMPL_HPP
#define SE_POINT_CLOUD_IO_IMPL_HPP

int save_point_cloud_vtk(se::Image<Eigen::Vector3f>& point_cloud, const std::string& filename, const Eigen::Matrix4f& T_WC)
{
    // Open the file for writing.
    std::ofstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "Unable to write file " << filename << "\n";
        return 1;
    }

    file << "# vtk DataFile Version 1.0\n";
    file << "vtk mesh generated from KFusion\n";
    file << "ASCII\n";
    file << "DATASET POLYDATA\n";
    file << "POINTS " << point_cloud.size() << " FLOAT\n";

    // Write the point data.
    for (size_t i = 0; i < point_cloud.size(); ++i) {
        const Eigen::Vector3f point_W = (T_WC * point_cloud[i].homogeneous()).head<3>();
        file << point_W.x() << " " << point_W.y() << " " << point_W.z() << "\n";
    }

    file.close();
    return 0;
}

#endif // SE_POINT_CLOUD_IO_IMPL_HPP
