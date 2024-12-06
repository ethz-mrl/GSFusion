/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.hpp"

#include "se/common/filesystem.hpp"
#include "se/common/str_utils.hpp"
#include "se/common/yaml.hpp"


std::string process_path(const std::string& path, const std::string& dataset_dir)
{
    // Expand leading ~ in the path if possible and interpret it as relative to dataset_dir if it's
    // a relative path.
    if (!path.empty()) {
        return se::str_utils::resolve_relative_path(se::str_utils::expand_user(path), dataset_dir);
    }
    return path;
}

namespace se {

void AppConfig::readYaml(const std::string& filename)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << filename << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << filename << "\n";
        return;
    }

    // Get the node containing the app configuration.
    const cv::FileNode node = fs["app"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default app configuration, no \"app\" section found in " << filename << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_bool(node, "enable_ground_truth", enable_ground_truth);
    se::yaml::subnode_as_string(node, "optim_params_path", optim_params_path);
    se::yaml::subnode_as_string(node, "ply_path", ply_path);
    se::yaml::subnode_as_string(node, "mesh_path", mesh_path);
    se::yaml::subnode_as_string(node, "slice_path", slice_path);
    se::yaml::subnode_as_string(node, "structure_path", structure_path);
    se::yaml::subnode_as_int(node, "integration_rate", integration_rate);
    se::yaml::subnode_as_int(node, "rendering_rate", rendering_rate);
    se::yaml::subnode_as_int(node, "meshing_rate", meshing_rate);
    se::yaml::subnode_as_int(node, "max_frames", max_frames);
    se::yaml::subnode_as_string(node, "log_file", log_file);

    const stdfs::path dataset_dir = stdfs::path(filename).parent_path();
    optim_params_path = process_path(optim_params_path, dataset_dir);
    ply_path = process_path(ply_path, dataset_dir);
    mesh_path = process_path(mesh_path, dataset_dir);
    slice_path = process_path(slice_path, dataset_dir);
    structure_path = process_path(structure_path, dataset_dir);
    log_file = process_path(log_file, dataset_dir);
}


std::ostream& operator<<(std::ostream& os, const AppConfig& c)
{
    os << str_utils::bool_to_pretty_str(c.enable_ground_truth, "enable_ground_truth") << "\n";
    os << str_utils::str_to_pretty_str(c.mesh_path, "mesh_path") << "\n";
    os << str_utils::str_to_pretty_str(c.slice_path, "slice_path") << "\n";
    os << str_utils::str_to_pretty_str(c.structure_path, "structure_path") << "\n";
    os << str_utils::value_to_pretty_str(c.integration_rate, "integration_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.rendering_rate, "rendering_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.meshing_rate, "meshing_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.max_frames, "max_frames") << "\n";
    os << str_utils::str_to_pretty_str(c.log_file, "log_file") << "\n";
    return os;
}
} // namespace se
