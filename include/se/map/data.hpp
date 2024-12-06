/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_HPP
#define SE_DATA_HPP

#include "data_colour.hpp"
#include "data_field.hpp"
#include "data_semantics.hpp"
#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

template<Field FldT = Field::TSDF, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct Data : public FieldData<FldT>, ColourData<ColB>, SemanticData<SemB> {
    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;
};


///////////////////
/// DELTA DATA  ///
///////////////////

template<Field FldT = Field::TSDF, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct DeltaData : public FieldDeltaData<FldT>, ColourDeltaData<ColB>, SemanticDeltaData<SemB> {
    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;
};


///////////////////
/// DATA CONFIG ///
///////////////////

template<Field FldT = Field::TSDF, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct DataConfig : public FieldDataConfig<FldT>, ColourDataConfig<ColB>, SemanticDataConfig<SemB> {
    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;

    /** Initializes all sub-configs to their sensible defaults.
     */
    DataConfig()
    {
    }

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in DataConfig::DataConfig().
     */
    DataConfig(const std::string& yaml_file) : FieldDataConfig<FldT>(yaml_file), ColourDataConfig<ColB>(yaml_file), SemanticDataConfig<SemB>(yaml_file)
    {
    }
};

template<Field FldT, Colour ColB, Semantics SemB>
std::ostream& operator<<(std::ostream& os, const DataConfig<FldT, ColB, SemB>& c)
{
    // Call the operator<< of the base classes.
    os << *static_cast<const FieldDataConfig<FldT>*>(&c);
    os << *static_cast<const ColourDataConfig<ColB>*>(&c);
    os << *static_cast<const SemanticDataConfig<SemB>*>(&c);
    return os;
}


template<Field FldT, Colour ColB, Semantics SemB>
inline void set_invalid(Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline void set_invalid(Data<Field::TSDF, ColB, SemB>& data)
{
    data = Data<Field::TSDF, ColB, SemB>();
}

template<Colour ColB, Semantics SemB>
inline void set_invalid(Data<Field::Occupancy, ColB, SemB>& data)
{
    data = Data<Field::Occupancy, ColB, SemB>();
}


template<Field FldT, Colour ColB, Semantics SemB>
inline bool is_valid(const Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline bool is_valid(const Data<Field::TSDF, ColB, SemB>& data)
{
    return data.weight != Data<Field::TSDF, ColB, SemB>{}.weight;
}

template<Colour ColB, Semantics SemB>
inline bool is_valid(const Data<Field::Occupancy, ColB, SemB>& data)
{
    return data.weight != Data<Field::Occupancy, ColB, SemB>{}.weight;
}


template<Field FldT, Colour ColB, Semantics SemB>
inline float get_field(const Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline float get_field(const Data<Field::TSDF, ColB, SemB>& data)
{
    return data.tsdf / static_cast<float>(tsdf_t_scale);
}

template<Colour ColB, Semantics SemB>
inline float get_field(const Data<Field::Occupancy, ColB, SemB>& data)
{
    return data.occupancy;
}


template<Field FldT, Colour ColB, Semantics SemB>
inline bool is_inside(const Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline bool is_inside(const Data<Field::TSDF, ColB, SemB>& data)
{
    return data.tsdf < 0;
}

template<Colour ColB, Semantics SemB>
inline bool is_inside(const Data<Field::Occupancy, ColB, SemB>& data)
{
    return data.occupancy > 0;
}


// Occupancy data setups
typedef Data<Field::Occupancy, Colour::Off, Semantics::Off> OccupancyData;
typedef Data<Field::Occupancy, Colour::On, Semantics::Off> OccupancyColData;
typedef Data<Field::Occupancy, Colour::Off, Semantics::NYUv2> OccupancyNYUv2Data;
typedef Data<Field::Occupancy, Colour::On, Semantics::NYUv2> OccupancyColNYUv2Data;
typedef Data<Field::Occupancy, Colour::Off, Semantics::MP3D> OccupancyMP3DData;
typedef Data<Field::Occupancy, Colour::On, Semantics::MP3D> OccupancyColMP3DData;
typedef Data<Field::Occupancy, Colour::Off, Semantics::COCO> OccupancyCOCOData;
typedef Data<Field::Occupancy, Colour::On, Semantics::COCO> OccupancyColCOCOData;
typedef Data<Field::Occupancy, Colour::Off, Semantics::Replica> OccupancyReplicaData;
typedef Data<Field::Occupancy, Colour::On, Semantics::Replica> OccupancyColReplicaData;

typedef DataConfig<Field::Occupancy, Colour::Off, Semantics::Off> OccupancyDataConfig;
typedef DataConfig<Field::Occupancy, Colour::On, Semantics::Off> OccupancyColDataConfig;
typedef DataConfig<Field::Occupancy, Colour::Off, Semantics::NYUv2> OccupancyNYUv2DataConfig;
typedef DataConfig<Field::Occupancy, Colour::On, Semantics::NYUv2> OccupancyColNYUv2DataConfig;
typedef DataConfig<Field::Occupancy, Colour::Off, Semantics::MP3D> OccupancyMP3DDataConfig;
typedef DataConfig<Field::Occupancy, Colour::On, Semantics::MP3D> OccupancyColMP3DDataConfig;
typedef DataConfig<Field::Occupancy, Colour::Off, Semantics::COCO> OccupancyCOCODataConfig;
typedef DataConfig<Field::Occupancy, Colour::On, Semantics::COCO> OccupancyColCOCODataConfig;
typedef DataConfig<Field::Occupancy, Colour::Off, Semantics::Replica> OccupancyReplicaDataConfig;
typedef DataConfig<Field::Occupancy, Colour::On, Semantics::Replica> OccupancyColReplicaDataConfig;

// TSDF data setups
typedef Data<Field::TSDF, Colour::Off, Semantics::Off> TSDFData;
typedef Data<Field::TSDF, Colour::On, Semantics::Off> TSDFColData;
typedef Data<Field::TSDF, Colour::Off, Semantics::NYUv2> TSDFNYUv2Data;
typedef Data<Field::TSDF, Colour::On, Semantics::NYUv2> TSDFColNYUv2Data;
typedef Data<Field::TSDF, Colour::Off, Semantics::MP3D> TSDFMP3DData;
typedef Data<Field::TSDF, Colour::On, Semantics::MP3D> TSDFColMP3DData;
typedef Data<Field::TSDF, Colour::Off, Semantics::COCO> TSDFCOCOData;
typedef Data<Field::TSDF, Colour::On, Semantics::COCO> TSDFColCOCOData;
typedef Data<Field::TSDF, Colour::Off, Semantics::Replica> TSDFReplicaData;
typedef Data<Field::TSDF, Colour::On, Semantics::Replica> TSDFColReplicaData;

typedef DataConfig<Field::TSDF, Colour::Off, Semantics::Off> TSDFDataConfig;
typedef DataConfig<Field::TSDF, Colour::On, Semantics::Off> TSDFColDataConfig;
typedef DataConfig<Field::TSDF, Colour::Off, Semantics::NYUv2> TSDFNYUv2DataConfig;
typedef DataConfig<Field::TSDF, Colour::On, Semantics::NYUv2> TSDFColNYUv2DataConfig;
typedef DataConfig<Field::TSDF, Colour::Off, Semantics::MP3D> TSDFMP3DDataConfig;
typedef DataConfig<Field::TSDF, Colour::On, Semantics::MP3D> TSDFColMP3DDataConfig;
typedef DataConfig<Field::TSDF, Colour::Off, Semantics::COCO> TSDFCOCODataConfig;
typedef DataConfig<Field::TSDF, Colour::On, Semantics::COCO> TSDFColCOCODataConfig;
typedef DataConfig<Field::TSDF, Colour::Off, Semantics::Replica> TSDFReplicaDataConfig;
typedef DataConfig<Field::TSDF, Colour::On, Semantics::Replica> TSDFColReplicaDataConfig;

} // namespace se

#endif // SE_DATA_HPP
