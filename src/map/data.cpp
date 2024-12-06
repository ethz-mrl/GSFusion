/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/data.hpp"

#include <cmath>

#include "se/common/str_utils.hpp"
#include "se/common/yaml.hpp"

namespace se {

FieldDataConfig<Field::TSDF>::FieldDataConfig() : truncation_boundary_factor(8), max_weight(100)
{
}


FieldDataConfig<Field::TSDF>::FieldDataConfig(const std::string& yaml_file) : FieldDataConfig<Field::TSDF>::FieldDataConfig()
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
        return;
    }

    // Get the node containing the data configuration.
    const cv::FileNode node = fs["data"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default data configuration, no \"data\" section found in " << yaml_file << "\n";
        return;
    }

    se::yaml::subnode_as_float(node, "truncation_boundary_factor", truncation_boundary_factor);
    // Ensure an integer max_weight is provided even in a float is used as weight_t.
    int max_weight_int = max_weight;
    se::yaml::subnode_as_int(node, "max_weight", max_weight_int);
    max_weight = max_weight_int;
}


std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::TSDF>& c)
{
    os << str_utils::value_to_pretty_str(c.truncation_boundary_factor, "truncation_boundary_factor") << "x\n";
    os << str_utils::value_to_pretty_str(c.max_weight, "max_weight") << "\n";
    return os;
}


ColourDataConfig<se::Colour::On>::ColourDataConfig()
{
}


ColourDataConfig<se::Colour::On>::ColourDataConfig(const std::string& /* yaml_file */) : ColourDataConfig<se::Colour::On>::ColourDataConfig()
{
}


std::ostream& operator<<(std::ostream& os, const ColourDataConfig<se::Colour::On>& /* c */)
{
    return os;
}


SemanticDataConfig<se::Semantics::Off>::SemanticDataConfig()
{
}


SemanticDataConfig<se::Semantics::Off>::SemanticDataConfig(const std::string& /* yaml_file */) : SemanticDataConfig<se::Semantics::Off>::SemanticDataConfig()
{
}


std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<se::Semantics::Off>& /* c */)
{
    return os;
}
} // namespace se
