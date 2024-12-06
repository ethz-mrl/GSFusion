/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MARCHING_CUBE_HPP
#define SE_MARCHING_CUBE_HPP


#include "edge_tables.hpp"
#include "se/common/timings.hpp"
#include "se/map/algorithms/mesh.hpp"
#include "se/map/octree/fetcher.hpp"
#include "se/map/octree/iterator.hpp"
#include "se/map/octree/visitor.hpp"


namespace se {
namespace meshing {

/// Single-res marching cube implementation

template<typename OctreeT>
inline Eigen::Vector3f compute_intersection(const OctreeT& octree, const Eigen::Vector3i& source_coord, const Eigen::Vector3i& dest_coord);

template<typename OctreeT>
inline Eigen::Vector3f interp_vertexes(const OctreeT& octree, const unsigned x, const unsigned y, const unsigned z, const int edge);

template<typename BlockT>
inline void gather_data(const BlockT* block, typename BlockT::DataType data[8], const int x, const int y, const int z);

template<typename OctreeT>
inline void gather_data(const OctreeT& octree, typename OctreeT::DataType data[8], const int x, const int y, const int z);

template<typename OctreeT>
uint8_t compute_index(const OctreeT& octree, const typename OctreeT::BlockType* block_ptr, const unsigned x, const unsigned y, const unsigned z);


/// Multires-res marching cube implementation

inline Eigen::Vector3f compute_dual_intersection(const float value_0,
                                                 const float value_1,
                                                 const Eigen::Vector3f& dual_corner_coord_0,
                                                 const Eigen::Vector3f& dual_corner_coord_1,
                                                 const float voxel_dim,
                                                 const int /* edge_case */);

template<typename DataT, typename ValueSelector>
inline Eigen::Vector3f interp_dual_vertexes(const int edge,
                                            const DataT data[8],
                                            const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f,
                                            const float voxel_dim,
                                            ValueSelector select_value);

/*
 * Normalised offsets of the dual corners from the primal corner
 */
static const Eigen::Vector3f norm_dual_offset_f[8] = {{-1, -1, -1}, {+1, -1, -1}, {+1, -1, +1}, {-1, -1, +1}, {-1, +1, -1}, {+1, +1, -1}, {+1, +1, +1}, {-1, +1, +1}};

template<typename BlockT, typename DataT>
inline void gather_dual_data(const BlockT* block,
                             const int scale,
                             const Eigen::Vector3f& primal_corner_coord_f,
                             DataT data[8],
                             std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f);

/*! \brief The following strategy is derived from I. Wald, A Simple, General,
 *  and GPU Friendly Method for Computing Dual Mesh and Iso-Surfaces of Adaptive Mesh Refinement (AMR) Data, 2020
 *
 * We validate the scale of all neighbouring blocks need to access the 8 dual corners for each primal corner
 * For each we compute the dual coordinates for primal coordinates with a relative block offset x,y,z in [0, block_size]
 * Due to the fact that block_size is still contained in the offset (rather than [0, block_size - 1], each primal corner
 * is contained in 1 (inside block), 2 (face), 4 (edge) or 8 (corner) neighbouring blocks. We prioritse the block neighbours
 * based on their value (heigher value = higher priority), where the value is calculated via
 * v = 4 << (x is +1) + 2 << (y is +1) + 1 << (z is +1).
 * The minium value is computed for all neighbours for dual corners (c0-8). This means if multiple dual corners fall
 * inside the same neighbouring block we take the minimum value of all the dual corners inside the block.
 * The threshold for populate the lower or higher priority list is the lowest dual corner cost of the corners falling
 * inside the block the relative primal coordinate belongs to.
 *
 * \param[in] primal_corner_coord_rel     relative voxel offset of the primal corner from the block coordinates
 * \param[in] block_size                  size of a voxel block in voxel units
 * \param[in] lower_priority_neighbours   blocks with lower priority, i.e. will be neglected if scale >= scale neighbour
 * \param[in] higher_priority_neighbours  blocks with higher priority, i.e. will only be neglected if scale > scale neighbour
 * \param[in] neighbours                  vector containing a vector with all corner offsets for a neighbouring block.
 *                                        First index is the main block.
 */
inline void norm_dual_corner_idxs(const Eigen::Vector3i& primal_corner_coord_rel,
                                  const int block_size,
                                  std::vector<int>& lower_priority_neighbours,
                                  std::vector<int>& higher_priority_neighbours,
                                  std::vector<std::vector<int>>& neighbours);

/*
 * Normalised offsets of the primal corners from the primal corner
 * The correct coordinates would be {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, ...
 * However this would cause a lot of extra computations to cast the voxels back and forth and
 * to make sure that the absolute coordinates {-0.5, y, z} won't be casted to {0, y, z} (and for y, z accordingly).
 */
static const Eigen::Vector3i logical_dual_offset[8] = {{-1, -1, -1}, {+0, -1, -1}, {+0, -1, +0}, {-1, -1, +0}, {-1, +0, -1}, {+0, +0, -1}, {+0, +0, +0}, {-1, +0, +0}};

template<typename OctreeT, typename DataT>
inline void gather_dual_data(const OctreeT& octree,
                             const typename OctreeT::BlockType* block,
                             const int scale,
                             const Eigen::Vector3i& primal_corner_coord,
                             DataT data[8],
                             std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f);

template<typename OctreeT, typename DataT>
void compute_dual_index(const OctreeT& octree,
                        const typename OctreeT::BlockType* block_ptr,
                        const int scale,
                        const Eigen::Vector3i& primal_corner_coord,
                        uint8_t& edge_pattern_idx,
                        DataT data[8],
                        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f);

inline bool checkVertex(const Eigen::Vector3f& vertex_M, const float dim);

} // namespace meshing


namespace algorithms {


template<typename OctreeT>
typename OctreeT::MeshType marching_cube_kernel(OctreeT& octree, std::vector<typename OctreeT::BlockType*>& block_ptrs);

template<typename OctreeT>
typename OctreeT::MeshType dual_marching_cube_kernel(OctreeT& octree, std::vector<typename OctreeT::BlockType*>& block_ptrs);


/**
 * \brief Generate the triangle mesh using a primal grid marching cube algorithm.
 *
 * \tparam OctreeT
 * \param[in]  octree       The octree to extract the mesh from
 * \param[out] triangles    The extracted mesh
 */
template<typename OctreeT>
typename OctreeT::MeshType marching_cube(OctreeT& octree);

/**
 * \brief Generate the triangle mesh using a primal grid marching cube algorithm.
 *        The algorithm only considers voxel values updated after the provided frame number.
 *
 * \tparam OctreeT
 * \param[in]  octree       The octree to extract the mesh from
 * \param[out] triangles    The extracted mesh
 * \param[in]  frame        The lower frame threshold of voxel values to consider
 */
template<typename OctreeT>
typename OctreeT::MeshType marching_cube(OctreeT& octree, const int frame);

/**
 * \brief Generate the triangle mesh using a dual grid marching cube algorithm.
 *
 * \tparam OctreeT
 * \param[in]  octree       The octree to extract the mesh from
 * \param[out] triangles    The extracted mesh
 */
template<typename OctreeT>
typename OctreeT::MeshType dual_marching_cube(OctreeT& octree);

/**
 * \brief Generate the triangle mesh using a dual grid marching cube algorithm.
 *        The algorithm only considers voxel values updated after the provided frame number.
 *
 * \tparam OctreeT
 * \param[in]  octree       The octree to extract the mesh from
 * \param[out] triangles    The extracted mesh
 * \param[in]  frame        The lower frame threshold of voxel values to consider
 */
template<typename OctreeT>
typename OctreeT::MeshType dual_marching_cube(OctreeT& octree, const int frame);

} // namespace algorithms
} // namespace se

#include "impl/marching_cube_impl.hpp"

#endif // SE_MARCHING_CUBE_HPP
