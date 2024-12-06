/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_FIELD_HPP
#define SE_DATA_FIELD_HPP

#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

template<Field FieldT>
struct FieldData {
};

template<>
struct FieldData<Field::TSDF> {
    tsdf_t tsdf = 1 * tsdf_t_scale;
    weight_t weight = 0;
    static constexpr bool invert_normals = true;
};

///////////////////
/// DELTA DATA  ///
///////////////////

template<Field FieldT>
struct FieldDeltaData {
};

template<>
struct FieldDeltaData<Field::TSDF> {
    delta_tsdf_t delta_tsdf = 0;
    delta_weight_t delta_weight = 0;
};

///////////////////
/// DATA CONFIG ///
///////////////////

enum class UncertaintyModel { Linear, Quadratic };

template<Field FieldT>
struct FieldDataConfig {
};

template<>
struct FieldDataConfig<Field::TSDF> {
    float truncation_boundary_factor;
    weight_t max_weight;

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::TSDF>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr Field FldT = Field::TSDF;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<Field::TSDF>& c);

} // namespace se

#endif // SE_DATA_FIELD_HPP
