/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_SEMANTICS_HPP
#define SE_DATA_SEMANTICS_HPP

#include "utils/confidence.hpp"
#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

// Semantic data
template<Semantics SemB>
struct SemanticData {
    static constexpr size_t num_classes = static_cast<size_t>(SemB);
    Confidence<num_classes> sem = {};
    weight_t sem_weight = 0;
};

// Semantic data
template<>
struct SemanticData<Semantics::Off> {
    static constexpr size_t num_classes = 0;
};


///////////////////
/// DELTA DATA  ///
///////////////////

template<Semantics SemB>
struct SemanticDeltaData {
    Confidence16s<static_cast<size_t>(SemB)> delta_sem = {};
    delta_weight_t delta_sem_weight = 0;
};

template<>
struct SemanticDeltaData<Semantics::Off> {
};


///////////////////
/// DATA CONFIG ///
///////////////////

// Semantic data
template<Semantics SemB>
struct SemanticDataConfig {
    /** Initializes the config to some sensible defaults.
     */
    SemanticDataConfig()
    {
    }

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in SemanticDataConfig<se::Semantics::On>SemanticDataConfig().
     */
    SemanticDataConfig(const std::string& /* yaml_file */)
    {
    }
};

template<Semantics SemB>
std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<SemB>& /* c */)
{
    return os;
}

// Semantic data
template<>
struct SemanticDataConfig<Semantics::Off> {
    SemanticDataConfig();
    SemanticDataConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<Semantics::Off>& c);

} // namespace se

#endif // SE_DATA_SEMANTICS_HPP
