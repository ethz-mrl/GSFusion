/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_CONFIG_HPP
#define SE_CONFIG_HPP


#include "reader.hpp"
#include "se/map/map.hpp"
#include "se/sensor/sensor.hpp"


namespace se {


struct AppConfig {
    /** Whether to use the available ground truth camera pose.
     */
    bool enable_ground_truth = false;

    /** The path where optimization parameters are saved.
     */
    std::string optim_params_path;

    /** The path where point clouds are saved.
     */
    std::string ply_path;

    /** The path where meshes are saved. Set to the empty string to disable meshing. Set to `"."`
     * for the current directory.
     */
    std::string mesh_path;

    /** The path where slice meshes are saved. Set to the empty string to disable slice meshing. Set
     * to `"."` for the current directory.
     */
    std::string slice_path;

    /** The path where structure meshes are saved. Set to the empty string to disable structure
     * meshing. Set to `"."` for the current directory.
     */
    std::string structure_path;

    /** Integrate a 3D reconstruction every integration_rate frames.
     */
    int integration_rate = 1;

    /** Render the 3D reconstruction every rendering_rate frames.
     *
     * \note AppConfig::enable_render == true (default) required.
     *
     * Special cases:
     * If rendering_rate == 0 the volume is only rendered for configuration::max_frame.
     * If rendering_rate < 0  the volume is only rendered for frame abs(rendering_rate).
     */
    int rendering_rate = 4;

    /** Mesh the 3D reconstruction every meshing_rate frames.
     *
     * Special cases:
     * If meshing_rate == 0 the volume is only meshed for configuration::max_frame.
     * If meshing_rate < 0  the volume is only meshed for frame abs(meshing_rate).
     */
    int meshing_rate = 100;

    /** The maximum number of frames to read. Set to -1 to read the whole dataset.
     */
    int max_frames = -1;

    /** The file where the timing results will be written to. The timing results will be written to
     * standard output if log_file is empty.
     *
     * \todo Show on stdout on empty string. Will have to update the perfstats.
     */
    std::string log_file;

    /** Reads the struct members from the "app" node of a YAML file. Members not present in the
     * YAML file aren't modified.
     */
    void readYaml(const std::string& filename);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const AppConfig& c);


template<typename DataConfigT, typename SensorConfigT>
struct Config {
    MapConfig map;
    DataConfigT data;
    SensorConfigT sensor;
    ReaderConfig reader;
    AppConfig app;

    /** Default initializes all configs.
     */
    Config();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in Config::Config().
     */
    Config(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


template<typename DataConfigT, typename SensorConfigT>
std::ostream& operator<<(std::ostream& os, const Config<DataConfigT, SensorConfigT>& c);


} // namespace se

#include "impl/config_impl.hpp"

#endif // SE_CONFIG_HPP
