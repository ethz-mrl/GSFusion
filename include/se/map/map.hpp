/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2023 Nils Funk
 * SPDX-FileCopyrightText: 2019-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_HPP
#define SE_MAP_HPP

#include <Eigen/StdVector>
#include <optional>

#include "se/common/math_util.hpp"
#include "se/common/str_utils.hpp"
#include "se/map/algorithms/marching_cube.hpp"
#include "se/map/algorithms/structure_meshing.hpp"
#include "se/map/data.hpp"
#include "se/map/io/mesh_io.hpp"
#include "se/map/io/octree_io.hpp"
#include "se/map/octree/visitor.hpp"
#include "se/map/raycaster.hpp"
#include "se/map/utils/octant_util.hpp"


namespace se {

struct MapConfig {
    /** The dimensions of the map in metres.
     */
    Eigen::Vector3f dim = Eigen::Vector3f::Constant(10.0f);

    /** The resolution of map voxels in metres.
     */
    float res = 0.1f;

    /** The transformation from the world frame W to the map frame M.
     */
    Eigen::Matrix4f T_MW = math::to_transformation((dim / 2).eval());

    /** Reads the struct members from the "map" node of a YAML file. Members not present in the YAML
     * file aren't modified.
     */
    void readYaml(const std::string& yaml_file);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const MapConfig& c);


// Forward Declaration
template<typename DataT = se::Data<Field::TSDF, Colour::Off, Semantics::Off>, Res ResT = Res::Single, int BlockSize = 8>
class Map;


template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
class Map<se::Data<FldT, ColB, SemB>, ResT, BlockSize> {
    public:
    typedef Data<FldT, ColB, SemB> DataType;
    typedef DataConfig<FldT, ColB, SemB> DataConfigType;
    typedef se::Octree<DataType, ResT, BlockSize> OctreeType;

    /**
     * \brief The map constructor.
     *
     * \note Default map origin is set at the centre of the map.
     *
     * \param[in] dim             The dimension of the map in [meter]
     * \param[in] res             The resolution of the map in [meter/voxel]
     * \param[in] data_config     The configuration file for the data
     */
    Map(const Eigen::Vector3f& dim, const float res, const DataConfig<FldT, ColB, SemB>& data_config = DataConfig<FldT, ColB, SemB>());

    /**
     * \brief The map constructor.
     *
     * \param map_config          The configuration file for the map (e.g. map dimension, resolution and origin)
     * \param data_config         The configuration file for the data
     */
    Map(const MapConfig& map_config, const DataConfig<FldT, ColB, SemB>& data_config = DataConfig<FldT, ColB, SemB>());

    /**
     * \brief Verify if a point is inside the map.
     *
     * \param[in] point_W         The point to be verified
     * \return True if the point is inside the map, false otherwise
     */
    bool contains(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the transformation from world to map frame
     *
     * \return T_MW
     */
    const Eigen::Matrix4f& getTMW() const
    {
        return T_MW_;
    };

    /**
     * \brief Get the transformation from map to world frame
     *
     * \return T_WM
     */
    const Eigen::Matrix4f& getTWM() const
    {
        return T_WM_;
    };

    /**
     * \brief Get the translation from world to map frame
     *
     * \return t_MW
     */
    Eigen::Vector3f gettMW() const
    {
        return se::math::to_translation(T_MW_);
    }

    /**
     * \brief Get the translation from map to world frame
     *
     * \return t_WM
     */
    Eigen::Vector3f gettWM() const
    {
        return se::math::to_translation(T_WM_);
    }

    /**
     * \brief Get the rotation from world to map frame
     *
     * \return R_MW
     */
    Eigen::Matrix3f getRMW() const
    {
        return se::math::to_rotation(T_MW_);
    }

    /**
     * \brief Get the rotation from map to world frame
     *
     * \return R_WM
     */
    Eigen::Matrix3f getRWM() const
    {
        return se::math::to_rotation(T_WM_);
    }

    /**
     * \brief Get the dimensions of the map in [meter] (length x width x height)
     *
     * \return The dimensions of the map
     */
    const Eigen::Vector3f& getDim() const
    {
        return dimension_;
    }

    /**
     * \brief Get the resolution of the map in [meter/voxel]
     *
     * \return The resolution of the map
     */
    float getRes() const
    {
        return resolution_;
    }

    /**
     * \brief Get the data configuration of the map.
     *
     * \return The data configuration of the map
     */
    const DataConfigType& getDataConfig() const
    {
        return data_config_;
    }

    /**
     * \brief Get the stored data at the provided coordinates in [meter].
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in]  point_W    The coordinates of the point in world frame [meter] to evaluate
     * \return The data at the provided coordinates
     */
    template<Safe SafeB = Safe::On>
    DataType getData(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the stored max data at the provided coordinates in [meter] for a given scale.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy      The dummy parameter disabling the function off for single res and TSDF maps // TODO: Clean up with C++20 using required
     * \param point_W         The coordinates of the point in world frame [meter] to accessed
     * \param scale_desired   The scale to be accessed
     * \return The max data at the provided coordinates and scale
     */
    template<Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, DataType> getMaxData(const Eigen::Vector3f& point_W, const int scale_desired) const
    {
        Eigen::Vector3i voxel_coord;

        if constexpr (SafeB == Safe::Off) // Evaluate at compile time
        {
            pointToVoxel<Safe::Off>(point_W, voxel_coord);
        }
        else {
            if (!pointToVoxel<Safe::On>(point_W, voxel_coord)) {
                return DataType();
            }
        }

        return se::visitor::getMaxData(*octree_ptr_, voxel_coord, scale_desired);
    }


    /**
     * \brief Get the interpolated field value at the provided coordinates.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in] point_W     The coordinates of the point in world frame [meter] to accessed
     * \return                The interpolated field value at the coordinates
     */
    template<Safe SafeB = Safe::On>
    std::optional<float> getFieldInterp(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the interpolated field value at the provided coordinates and the scale it is stored at.
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy          The dummy parameter disabling the function off for single res maps // TODO: Clean up with C++20 using required
     * \param[in] point_W         The coordinates of the point in world frame [meter] to accessed
     * \param[out] returned_scale The scale the data is stored at
     * \return                    The interpolated field value at the coordinates
     */
    template<Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<float>> getFieldInterp(const Eigen::Vector3f& point_W, int& returned_scale) const;

    /** \brief Interpolate a value at the supplied coordinates.
     *
     * \tparam GetF           The type of the function returning the value to be interpolated.
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in] point_W     The coordinates of the point in world frame [meter] to accessed
     * \param[in] get_value   A function returning the value of type `T` to be interpolated with the
     *                        following prototype:
     *                        \code{.cpp}
     *                        template<typename OctreeT>
     *                        T get_value(const typename OctreeT::DataType& data);
     *                        \endcode
     * \return                The interpolated value if the data is valid, std::nullopt otherwise.
     */
    template<typename GetF, Safe SafeB = Safe::Off>
    std::optional<std::invoke_result_t<GetF, DataType>> getInterp(const Eigen::Vector3f& point_W, GetF get_value) const;

    /** \brief Interpolate a value at the supplied coordinates and return the scale it was
     * interpolated at.
     *
     * \tparam GetF               The type of the function returning the value to be interpolated.
     * \tparam SafeB              The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy          The dummy parameter disabling the function off for single res maps // TODO: Clean up with C++20 using required
     * \param[in] point_W         The coordinates of the point in world frame [meter] to accessed
     * \param[out] returned_scale The scale the data is stored at
     * \param[in] get_value       A function returning the value of type `T` to be interpolated with
     *                            the following prototype:
     *                            \code{.cpp}
     *                            template<typename OctreeT>
     *                            T get_value(const typename OctreeT::DataType& data);
     *                            \endcode
     * \return                    The interpolated value if the data is valid, std::nullopt
     *                            otherwise.
     */
    template<typename GetF, Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<std::invoke_result_t<GetF, DataType>>> getInterp(const Eigen::Vector3f& point_W, int& returned_scale, GetF get_value) const;

    /**
     * \brief Get the field gradient at the provided coordinates.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in] point_W     The coordinates of the point in world frame [meter] to accessed
     * \return                The filed gradient at the coordinates
     */
    template<Safe SafeB = Safe::On>
    std::optional<Eigen::Vector3f> getFieldGrad(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save three slices of the field value, each perpendicular to one of the axes (x, y and
     * z) at the provided coordinates. Setting any of the filenames to the empty string will skip
     * saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \return Zero on success and non-zero on error.
     */
    int saveFieldSlices(const std::string& filename_x, const std::string& filename_y, const std::string& filename_z, const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save three slices of the maximum field value, each perpendicular to one of the axes
     * (x, y and z) at the provided coordinates. Setting any of the filenames to the empty string
     * will skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \param[in] scale      The minimum scale the maximum field values will be extracted from.
     * \return Zero on success and non-zero on error.
     */
    template<se::Field FldTDummy = FldT>
    typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
    saveMaxFieldSlices(const std::string& filename_x, const std::string& filename_y, const std::string& filename_z, const Eigen::Vector3f& point_W, const int scale) const;

    /**
     * \brief Save three slices of the integration scale, each perpendicular to one of the axes (x,
     * y and z) at the provided coordinates. Setting any of the filenames to the empty string will
     * skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \return Zero on success and non-zero on error.
     */
    template<Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, int>
    saveScaleSlices(const std::string& filename_x, const std::string& filename_y, const std::string& filename_z, const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save the octree structure to a file.
     *
     * \param[in] filename The file where the mesh will be saved. The file format will be selected
     *                     based on the file extension. Its extension must be one of those in
     *                     se::io::mesh_extensions.
     * \param[in] T_WM     Transformation from the world frame where the mesh is generated to the world
     *                     frame. Defaults to identity.
     * \return Zero on success and non-zero on error.
     */
    int saveStructure(const std::string& filename, const Eigen::Matrix4f& T_WM = Eigen::Matrix4f::Identity()) const;

    /**
     * \brief Create a mesh in the world frame in units of metres.
     *
     * \param[in] T_OW     Transformation from the world frame in units of metres to the output
     *                     frame. Defaults to identity.
     * \return The created mesh.
     */
    typename OctreeType::MeshType mesh(const Eigen::Matrix4f& T_OW = Eigen::Matrix4f::Identity()) const;

    /**
     * \brief Create a mesh in the map frame in units of voxels.
     *
     * \return The created mesh.
     */
    typename OctreeType::MeshType meshVoxel() const;

    /**
     * \brief Create a mesh in the world frame in units of metres and save it to a file.
     *
     * \param[in] filename The file where the mesh will be saved. The file format will be selected
     *                     based on the file extension. Its extension must be one of those in
     *                     se::io::mesh_extensions.
     * \param[in] T_OW     Transformation from the world frame in units of metres to the output frame. Defaults to identity.
     * \return Zero on success and non-zero on error.
     */
    int saveMesh(const std::string& filename, const Eigen::Matrix4f& T_OW = Eigen::Matrix4f::Identity()) const;

    /**
     * \brief Create a mesh in the map frame in units of voxel and save it to a file.
     *
     * \param[in] filename The file where the mesh will be saved. The file format will be selected
     *                     based on the file extension. Its extension must be one of those in
     *                     se::io::mesh_extensions.
     * \return Zero on success and non-zero on error.
     */
    int saveMeshVoxel(const std::string& filename) const;

    /**
     * \brief Convert voxel coordinates in [voxel] to its centre point coordinates in [meter].
     *
     * \warning The function assumes the voxel has size 1 (i.e. scale 0).
     *
     * \param[in]  voxel_coord    The voxel coordinates in [voxel] to be converted
     * \param[out] point_W        The converted centre point coordinates in [meter]
     */
    void voxelToPoint(const Eigen::Vector3i& voxel_coord, Eigen::Vector3f& point_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its centre point coordinates in [meter].
     *
     * \param[in]  voxel_coord    The voxel coordinates in [voxel] to be converted
     * \param[in]  voxel_size     The size of the voxel in [voxel]
     * \param[out] point_W        The converted centre point coordinates in [meter]
     */
    void voxelToPoint(const Eigen::Vector3i& voxel_coord, const int voxel_size, Eigen::Vector3f& point_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its eight corner point coordinates in [meter].
     *
     * \warning The function assumes the voxel has size 1 (i.e. scale 0).
     *
     * \param[in]  voxel_coord        The voxel coordinates in [voxel] to be converted
     * \param[out] corner_points_W    The converted centre point coordinates in [meter]
     */
    void voxelToCornerPoints(const Eigen::Vector3i& voxel_coord, Eigen::Matrix<float, 3, 8>& corner_points_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its eight corner point coordinates in [meter].
     *
     * \param[in]  voxel_coord        The voxel coordinates in [voxel] to be converted
     * \param[in]  voxel_size         The size of the voxel in [voxel]
     * \param[out] corner_points_W    The converted centre point coordinates in [meter]
     */
    void voxelToCornerPoints(const Eigen::Vector3i& voxel_coord, const int voxel_size, Eigen::Matrix<float, 3, 8>& corner_points_W) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates (bottom, front, left corner) in [voxel]
     *
     * \note Use as `map. template pointToVoxel<se::Safe::Off>(...)
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord    The converted voxel coordinates (bottom, front, left corner) in [voxel]
     * \return True if the point inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    typename std::enable_if_t<SafeB == se::Safe::On, bool> pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates (bottom, front, left corner) in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord    The converted voxel coordinates (bottom, front, left corner) in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    typename std::enable_if_t<SafeB == se::Safe::Off, bool> pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord_f  The converted voxel coordinates in [voxel]
     * \return True if the point inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    typename std::enable_if_t<SafeB == se::Safe::On, bool> pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3f& voxel_coord_f) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord_f  The converted voxel coordinates in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    typename std::enable_if_t<SafeB == se::Safe::Off, bool> pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3f& voxel_coord_f) const;

    /**
     * \brief Convert a vector of point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  points_W       The vector of point coordinates in [meter] to be converted
     * \param[out] voxel_coords   The vector of converted voxel coordinates in [voxel]
     * \return True if all points are inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    typename std::enable_if_t<SafeB == se::Safe::On, bool> pointsToVoxels(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points_W,
                                                                          std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords) const;

    /**
     * \brief Convert a vector of point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  points_W       The vector of point coordinates in [meter] to be converted
     * \param[out] voxel_coords   The vector of converted voxel coordinates in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    typename std::enable_if_t<SafeB == se::Safe::Off, bool> pointsToVoxels(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points_W,
                                                                           std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords) const;

    /**
     * \brief Get the shared pointer to the octree.
     *
     * \return The shared pointer to the octree
     */
    std::shared_ptr<OctreeType> getOctree()
    {
        return octree_ptr_;
    };

    /**
     * \brief Get the const shared pointer to the octree.
     *
     * \return The const shared pointer to the octree
     */
    std::shared_ptr<OctreeType> getOctree() const
    {
        return octree_ptr_;
    };

    /**
     * \brief Update the octree pointer to a new one.
     *
     * \param[in] octree_ptr  The shared pointer to the new octree to be assigned
     */
    void setOctree(std::shared_ptr<OctreeType> octree_ptr)
    {
        octree_ptr_ = octree_ptr;
    };

    /** Return the axis-aligned bounding box in the world frame W of the map's allocated leaves.
     */
    Eigen::AlignedBox3f aabb() const;

    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;

    static constexpr Res res_ = ResT;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    std::shared_ptr<OctreeType> octree_ptr_;
    const float resolution_;          ///< The resolution of the map
    const Eigen::Vector3f dimension_; ///< The dimensions of the map
    const Eigen::Matrix4f T_MW_;      ///< The transformation from world to map frame
    const Eigen::Matrix4f T_WM_;      ///< The transformation from map to world frame

    const Eigen::Vector3f lb_M_; ///< The lower map bound
    const Eigen::Vector3f ub_M_; ///< The upper map bound

    const DataConfigType data_config_; ///< The configuration of the data

    /** The eight relative unit corner offsets */
    static const Eigen::Matrix<float, 3, 8> corner_rel_steps_;
};

//// Full alias template for alternative setup
template<se::Field FldT = se::Field::TSDF, se::Colour ColB = se::Colour::Off, se::Semantics SemB = se::Semantics::Off, se::Res ResT = se::Res::Single, int BlockSize = 8>
using MapD = Map<Data<FldT, ColB, SemB>, ResT, BlockSize>;


// Occupancy map setups
template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyMap = Map<OccupancyData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColMap = Map<OccupancyColData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyNYUv2Map = Map<OccupancyNYUv2Data, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColNYUv2Map = Map<OccupancyColNYUv2Data, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyMP3DMap = Map<OccupancyMP3DData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColMP3DMap = Map<OccupancyColMP3DData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyCOCOMap = Map<OccupancyCOCOData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColCOCOMap = Map<OccupancyColCOCOData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyReplicaMap = Map<OccupancyReplicaData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Multi, int BlockSize = 8>
using OccupancyColReplicaMap = Map<OccupancyColReplicaData, ResT, BlockSize>;


// TSDF map setups
template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFMap = Map<TSDFData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColMap = Map<TSDFColData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFNYUv2Map = Map<TSDFNYUv2Data, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColNYUv2Map = Map<TSDFColNYUv2Data, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFMP3DMap = Map<TSDFMP3DData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColMP3DMap = Map<TSDFColMP3DData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFCOCOMap = Map<TSDFCOCOData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColCOCOMap = Map<TSDFColCOCOData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFReplicaMap = Map<TSDFReplicaData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColReplicaMap = Map<TSDFColReplicaData, ResT, BlockSize>;

} // namespace se

#include "impl/map_impl.hpp"

#endif // SE_MAP_HPP
