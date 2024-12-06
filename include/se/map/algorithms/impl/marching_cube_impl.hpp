/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MARCHING_CUBE_IMPL_HPP
#define SE_MARCHING_CUBE_IMPL_HPP

namespace se {

namespace meshing {

/** Functors to select which isosurface to mesh using marching cubes. They return an index to
 * triTable given the data of the 8 neighbouring vertices. See also here:
 * https://en.wikipedia.org/wiki/Marching_cubes#Algorithm
 */
namespace isosurface {

/** Mesh the surface between occupied and free space. This is what's typically wanted when meshing.
 * The mesh faces are oriented away from occupied space.
 */
static constexpr auto occupied = [](const auto data[8]) -> std::uint8_t {
    // A face should be meshed if there are no unknown vertices (not valid) and if there is at least
    // 1 occupied vertex (valid and inside).
    std::uint8_t edge_index = 0;
    for (int i = 0; i < 8; i++) {
        if (!is_valid(data[i])) {
            return 0;
        }
        if (is_inside(data[i])) {
            edge_index |= 1 << i;
        }
    }
    return edge_index;
};

/** Mesh the surface between free and occupied or unknown space. This will create a mesh that
 * encloses all free space in the map. The mesh faces are oriented away from free space.
 */
static constexpr auto free = [](const auto data[8]) -> std::uint8_t {
    // A face should be meshed if there is at least 1 free vertex (valid and not inside). Since both
    // the boundaries between free and unknown, and free and occupied are needed there's no early
    // exit on invalid data.
    std::uint8_t edge_index = 0;
    for (int i = 0; i < 8; i++) {
        if (is_valid(data[i]) && !is_inside(data[i])) {
            edge_index |= 1 << i;
        }
    }
    return edge_index;
};

/** Mesh the surface between free and unknown space, also known as frontiers. The mesh faces are
 * oriented towards unknown space.
 */
static constexpr auto frontier = [](const auto data[8]) -> std::uint8_t {
    // Frontiers are the boundaries between free and unknown space. This means there should be at
    // least 1 free vertex (valid and not inside) and at least 1 unknown vertex (not valid) for a
    // face to be meshed.
    std::uint8_t edge_index = 0;
    int num_valid = 0;
    for (int i = 0; i < 8; i++) {
        if (is_valid(data[i])) {
            num_valid++;
            if (!is_inside(data[i])) {
                edge_index |= 1 << i;
            }
        }
    }
    return num_valid == 8 ? 0 : edge_index;
};

} // namespace isosurface


/// Single-res marching cube implementation

template<typename OctreeT>
inline Eigen::Vector3f compute_intersection(const OctreeT& octree, const Eigen::Vector3i& source_coord, const Eigen::Vector3i& dest_coord)
{
    typename OctreeT::DataType data_0 = se::visitor::getData(octree, source_coord);
    const float value_0 = get_field(data_0);

    typename OctreeT::DataType data_1 = se::visitor::getData(octree, dest_coord);
    const float value_1 = get_field(data_1);

    Eigen::Vector3f source_point_M = (source_coord.cast<float>() + se::sample_offset_frac);
    Eigen::Vector3f dest_point_M = (dest_coord.cast<float>() + se::sample_offset_frac);

    return source_point_M + (0.0 - value_0) * (dest_point_M - source_point_M) / (value_1 - value_0);
}

template<typename OctreeT>
inline Eigen::Vector3f interp_vertexes(const OctreeT& octree, const unsigned x, const unsigned y, const unsigned z, const int edge)
{
    switch (edge) {
    case 0:
        return compute_intersection(octree, Eigen::Vector3i(x, y, z), Eigen::Vector3i(x + 1, y, z));
    case 1:
        return compute_intersection(octree, Eigen::Vector3i(x + 1, y, z), Eigen::Vector3i(x + 1, y, z + 1));
    case 2:
        return compute_intersection(octree, Eigen::Vector3i(x + 1, y, z + 1), Eigen::Vector3i(x, y, z + 1));
    case 3:
        return compute_intersection(octree, Eigen::Vector3i(x, y, z), Eigen::Vector3i(x, y, z + 1));
    case 4:
        return compute_intersection(octree, Eigen::Vector3i(x, y + 1, z), Eigen::Vector3i(x + 1, y + 1, z));
    case 5:
        return compute_intersection(octree, Eigen::Vector3i(x + 1, y + 1, z), Eigen::Vector3i(x + 1, y + 1, z + 1));
    case 6:
        return compute_intersection(octree, Eigen::Vector3i(x + 1, y + 1, z + 1), Eigen::Vector3i(x, y + 1, z + 1));
    case 7:
        return compute_intersection(octree, Eigen::Vector3i(x, y + 1, z), Eigen::Vector3i(x, y + 1, z + 1));

    case 8:
        return compute_intersection(octree, Eigen::Vector3i(x, y, z), Eigen::Vector3i(x, y + 1, z));
    case 9:
        return compute_intersection(octree, Eigen::Vector3i(x + 1, y, z), Eigen::Vector3i(x + 1, y + 1, z));
    case 10:
        return compute_intersection(octree, Eigen::Vector3i(x + 1, y, z + 1), Eigen::Vector3i(x + 1, y + 1, z + 1));
    case 11:
        return compute_intersection(octree, Eigen::Vector3i(x, y, z + 1), Eigen::Vector3i(x, y + 1, z + 1));
    }
    return Eigen::Vector3f::Constant(0);
}

template<typename BlockT>
inline void gather_data(const BlockT* block_ptr, typename BlockT::DataType data_arr[8], const int x, const int y, const int z)
{
    data_arr[0] = block_ptr->getData(Eigen::Vector3i(x, y, z));
    data_arr[1] = block_ptr->getData(Eigen::Vector3i(x + 1, y, z));
    data_arr[2] = block_ptr->getData(Eigen::Vector3i(x + 1, y, z + 1));
    data_arr[3] = block_ptr->getData(Eigen::Vector3i(x, y, z + 1));
    data_arr[4] = block_ptr->getData(Eigen::Vector3i(x, y + 1, z));
    data_arr[5] = block_ptr->getData(Eigen::Vector3i(x + 1, y + 1, z));
    data_arr[6] = block_ptr->getData(Eigen::Vector3i(x + 1, y + 1, z + 1));
    data_arr[7] = block_ptr->getData(Eigen::Vector3i(x, y + 1, z + 1));
}


template<typename OctreeT>
inline void gather_data(const OctreeT& octree, typename OctreeT::DataType data[8], const int x, const int y, const int z)
{
    data[0] = se::visitor::getData(octree, Eigen::Vector3i(x, y, z));
    data[1] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y, z));
    data[2] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y, z + 1));
    data[3] = se::visitor::getData(octree, Eigen::Vector3i(x, y, z + 1));
    data[4] = se::visitor::getData(octree, Eigen::Vector3i(x, y + 1, z));
    data[5] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y + 1, z));
    data[6] = se::visitor::getData(octree, Eigen::Vector3i(x + 1, y + 1, z + 1));
    data[7] = se::visitor::getData(octree, Eigen::Vector3i(x, y + 1, z + 1));
}


template<typename OctreeT, typename DataToIndexF>
uint8_t compute_index(const OctreeT& octree, const typename OctreeT::BlockType* block_ptr, const unsigned x, const unsigned y, const unsigned z, DataToIndexF data_to_index)
{
    unsigned int block_size = block_ptr->getSize();
    unsigned int local = ((x % block_size == block_size - 1) << 2) | ((y % block_size == block_size - 1) << 1) | ((z % block_size) == block_size - 1);

    typename OctreeT::DataType data[8];
    if (!local) {
        gather_data(block_ptr, data, x, y, z);
    }
    else {
        gather_data(octree, data, x, y, z);
    }

    return data_to_index(data);
}


/// Multi-res marching cube implementation

inline Eigen::Vector3f
compute_dual_intersection(const float value_0, const float value_1, const Eigen::Vector3f& dual_corner_coord_0, const Eigen::Vector3f& dual_corner_coord_1, const int /* edge_case */)
{
    Eigen::Vector3f dual_point_0_M = dual_corner_coord_0;
    Eigen::Vector3f dual_point_1_M = dual_corner_coord_1;
    float iso_value = 0.f;
    return dual_point_0_M + (iso_value - value_0) * (dual_point_1_M - dual_point_0_M) / (value_1 - value_0);
}


template<typename DataT>
inline Eigen::Vector3f interp_dual_vertexes(const int edge, const DataT data[8], const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f)
{
    switch (edge) {
    case 0:
        return compute_dual_intersection(get_field(data[0]), get_field(data[1]), dual_corner_coords_f[0], dual_corner_coords_f[1], 0);
    case 1:
        return compute_dual_intersection(get_field(data[1]), get_field(data[2]), dual_corner_coords_f[1], dual_corner_coords_f[2], 1);
    case 2:
        return compute_dual_intersection(get_field(data[2]), get_field(data[3]), dual_corner_coords_f[2], dual_corner_coords_f[3], 2);
    case 3:
        return compute_dual_intersection(get_field(data[0]), get_field(data[3]), dual_corner_coords_f[0], dual_corner_coords_f[3], 3);
    case 4:
        return compute_dual_intersection(get_field(data[4]), get_field(data[5]), dual_corner_coords_f[4], dual_corner_coords_f[5], 4);
    case 5:
        return compute_dual_intersection(get_field(data[5]), get_field(data[6]), dual_corner_coords_f[5], dual_corner_coords_f[6], 5);
    case 6:
        return compute_dual_intersection(get_field(data[6]), get_field(data[7]), dual_corner_coords_f[6], dual_corner_coords_f[7], 6);
    case 7:
        return compute_dual_intersection(get_field(data[4]), get_field(data[7]), dual_corner_coords_f[4], dual_corner_coords_f[7], 7);
    case 8:
        return compute_dual_intersection(get_field(data[0]), get_field(data[4]), dual_corner_coords_f[0], dual_corner_coords_f[4], 8);
    case 9:
        return compute_dual_intersection(get_field(data[1]), get_field(data[5]), dual_corner_coords_f[1], dual_corner_coords_f[5], 9);
    case 10:
        return compute_dual_intersection(get_field(data[2]), get_field(data[6]), dual_corner_coords_f[2], dual_corner_coords_f[6], 10);
    case 11:
        return compute_dual_intersection(get_field(data[3]), get_field(data[7]), dual_corner_coords_f[3], dual_corner_coords_f[7], 11);
    }
    return Eigen::Vector3f::Constant(0);
}


template<typename BlockT, typename DataT>
inline void gather_dual_data(const BlockT* block_ptr,
                             const int scale,
                             const Eigen::Vector3f& primal_corner_coord_f,
                             DataT data_arr[8],
                             std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f)
{
    // In the local case:        actual_dual_offset = actual_dual_scaling * norm_dual_offset_f and
    // dual_corner_coords_f = primal_corner_coord_f + actual_dual_scaling * norm_dual_offset_f
    const float actual_dual_scaling = (float) (1 << scale) / 2;
    for (int corner_idx = 0; corner_idx < 8; corner_idx++) {
        dual_corner_coords_f[corner_idx] = primal_corner_coord_f + actual_dual_scaling * norm_dual_offset_f[corner_idx];
        data_arr[corner_idx] = block_ptr->getData(dual_corner_coords_f[corner_idx].cast<int>()); /// <- TODO: Should take data from current scale
    }
}

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
                                  std::vector<std::vector<int>>& neighbours)
{
    // 26 binary cases (6 faces, 8 corners, 12 edges)
    // 100 000 upper x crossing
    // 010 000 upper y crossing
    // 001 000 upper z crossing

    // 000 100 lower x crossing
    // 000 010 lower y crossing
    // 000 001 lower z crossing

    unsigned int crossmask = ((primal_corner_coord_rel.x() == block_size) << 5) | ((primal_corner_coord_rel.y() == block_size) << 4) | ((primal_corner_coord_rel.z() == block_size) << 3)
        | ((primal_corner_coord_rel.x() == 0) << 2) | ((primal_corner_coord_rel.y() == 0) << 1) | (primal_corner_coord_rel.z() == 0);

    switch (crossmask) {
    // 6 Faces
    case 1: /* CASE 1 = crosses lower z */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1} and (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1} and (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Higher priority
        // -
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {};
        neighbours = {{2, 3, 6, 7}, {0, 1, 4, 5}};
    } break;
    case 2: /* CASE 2 = crosses lower y */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Higher priority
        // -
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {};
        neighbours = {{4, 5, 6, 7}, {0, 1, 2, 3}};
    } break;
    case 4: /* CASE 3 = crosses lower x */
    {
        // Inside
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Higher priority
        // -
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {};
        neighbours = {{1, 2, 5, 6}, {0, 3, 4, 7}};
    } break;
    case 8: /* CASE 4 = crosses upper z */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1} and (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1} and (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {3};
        neighbours = {{0, 1, 4, 5}, {2, 3, 6, 7}};
    } break;
    case 16: /* CASE 5 = crosses upper y */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Lower priority
        // -
        // Higher priority
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {4};
        neighbours = {{0, 1, 2, 3}, {4, 5, 6, 7}};
    } break;
    case 32: /* CASE 6 = crosses upper x */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1} and (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Lower priority
        // -
        // Higher priority
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1} and (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1};
        neighbours = {{0, 3, 4, 7}, {1, 2, 5, 6}};
    } break;


        // 8 Corners
    case 7: /* CASE 7 = crosses lower x(4), y(2), z(1) */
    {
        // Inside
        // (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1},
        // Higher priority
        // -
        lower_priority_neighbours = {0, 1, 2, 3, 4, 5, 7};
        higher_priority_neighbours = {};
        neighbours = {{6}, {0}, {1}, {2}, {3}, {4}, {5}, {7}};
    } break;
    case 14: /* CASE 8 = crosses lower x(4), y(2) and upper z(8) */
    {
        // Inside
        // (c5;v6)-{+1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1},
        // (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1},
        // Higher priority
        // (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 1, 2, 3, 4, 7};
        higher_priority_neighbours = {6};
        neighbours = {{5}, {0}, {1}, {2}, {3}, {4}, {6}, {7}};
    } break;
    case 21: /* CASE 9 = crosses lower x(4), upper y(16) and lower z(1) */
    {
        // Inside
        // (c2;v5)-{+1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1},
        // (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1},
        // Higher priority
        // (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 1, 3, 4, 7};
        higher_priority_neighbours = {5, 6};
        neighbours = {{2}, {0}, {1}, {3}, {4}, {5}, {6}, {7}};
    } break;
    case 28: /* CASE 10 = crosses lower x(4) and upper y(16), z(8) */
    {
        // Inside
        // (c1;v4)-{+1, -1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1},
        // (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}
        // Higher priority
        // (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3, 4, 7};
        higher_priority_neighbours = {2, 5, 6};
        neighbours = {{1}, {0}, {2}, {3}, {4}, {5}, {6}, {7}};
    } break;
    case 35: /* CASE 11 = crosses upper x(32) and lower y(2), z(1) */
    {
        // Inside
        // (c7;v3)-{-1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1}
        // Higher priority
        // (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3, 4};
        higher_priority_neighbours = {1, 2, 5, 6};
        neighbours = {{7}, {0}, {1}, {2}, {3}, {4}, {5}, {6}};
    } break;
    case 42: /* CASE 12 = crosses upper x(32), lower y(2) and upper z(8) */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}, (c3;v1)-{-1, -1, +1}
        // Higher priority
        // (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1},
        // (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3};
        higher_priority_neighbours = {1, 2, 5, 6, 7};
        neighbours = {{4}, {0}, {1}, {2}, {3}, {5}, {6}, {7}};
    } break;
    case 49: /* CASE 13 = crosses upper x(32), y(16) and lower z(1) */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1}
        // Higher priority
        // (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}, (c1;v4)-{+1, -1, -1},
        // (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {1, 2, 4, 5, 6, 7};
        neighbours = {{3}, {0}, {1}, {2}, {4}, {5}, {6}, {7}};
    } break;
    case 56: /* CASE 14 = crosses upper x(32), y(16), z(8) */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1}, (c4;v2)-{-1, +1, -1}, (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1}, (c2;v5)-{+1, -1, +1}, (c5;v6)-{+1, +1, -1}, (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1, 2, 3, 4, 5, 6, 7};
        neighbours = {{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}};
    } break;


        // 12 Edges
    case 3: /* CASE 15 = crosses lower y(2), z(1) */
    {
        // Inside
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // Higher priority
        // -
        lower_priority_neighbours = {0, 3, 4};
        higher_priority_neighbours = {};
        neighbours = {{6, 7}, {0, 1}, {2, 3}, {4, 5}};
    } break;
    case 5: /* CASE 16 = crosses lower x(4), z(1) */
    {
        // Inside
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Higher priority
        // -
        lower_priority_neighbours = {0, 1, 3};
        higher_priority_neighbours = {};
        neighbours = {{2, 6}, {0, 4}, {3, 7}, {1, 5}};
    } break;
    case 6: /* CASE 17 = crosses lower x(4), y(2) */
    {
        // Inside
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Higher priority
        // -
        lower_priority_neighbours = {0, 1, 4};
        higher_priority_neighbours = {};
        neighbours = {{5, 6}, {0, 3}, {4, 7}, {1, 2}};
    } break;
    case 10: /* CASE 18 = crosses lower y(2) and upper z(8) */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        // Higher priority
        //
        lower_priority_neighbours = {0, 3};
        higher_priority_neighbours = {7};
        neighbours = {{4, 5}, {0, 1}, {2, 3}, {6, 7}};
    } break;
    case 12: /* CASE 19 = crosses lower x(4) and upper z(8) */
    {
        // Inside
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // Higher priority
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 3};
        higher_priority_neighbours = {2};
        neighbours = {{1, 5}, {0, 4}, {3, 7}, {2, 6}};
    } break;
    case 17: /* CASE 20 = crosses upper y(16) and lower z(1) */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // Higher priority
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {4, 7};
        neighbours = {{2, 3}, {0, 1}, {4, 5}, {6, 7}};
    } break;
    case 20: /* CASE 21 = crosses lower x(4) and upper y(16)*/
    {
        // Inside
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Higher priority
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0, 4};
        higher_priority_neighbours = {5};
        neighbours = {{1, 2}, {0, 3}, {4, 7}, {5, 6}};
    } break;
    case 24: /* CASE 22 = crosses upper y(16), z(8)*/
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c1;v4)-{+1, -1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1} and (c2;v5)-{+1, -1, +1}
        // (c4;v2)-{-1, +1, -1} and (c5;v6)-{+1, +1, -1}
        // (c7;v3)-{-1, +1, +1} and (c6;v7)-{+1, +1, +1}
        //
        lower_priority_neighbours = {};
        higher_priority_neighbours = {3, 4, 7};
        neighbours = {{0, 1}, {2, 3}, {4, 5}, {6, 7}};
    } break;
    case 33: /* CASE 23 = crosses upper x(32) and lower z(1) */
    {
        // Inside
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // Higher priority
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {1, 2};
        neighbours = {{3, 7}, {0, 4}, {1, 5}, {2, 6}};
    } break;
    case 34: /* CASE 24 = crosses upper x(32) and lower y(2) */
    {
        // Inside
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // Lower priority
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // Higher priority
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {0};
        higher_priority_neighbours = {1, 5};
        neighbours = {{4, 7}, {0, 3}, {1, 2}, {5, 6}};
    } break;
    case 40: /* CASE 25 = crosses upper x(32), z(8) */
    {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c4;v2)-{-1, +1, -1}
        // Lower priority
        // -
        // Higher priority
        // (c3;v1)-{-1, -1, +1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c5;v6)-{+1, +1, -1}
        // (c2;v5)-{+1, -1, +1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1, 2, 3};
        neighbours = {{0, 4}, {3, 7}, {1, 5}, {2, 6}};
    } break;
    case 48: /* CASE 26 = crosses upper x(32), y(16) */ {
        // Inside
        // (c0;v0)-{-1, -1, -1} and (c3;v1)-{-1, -1, +1}
        // Lower priority
        // -
        // Higher priority
        // (c4;v2)-{-1, +1, -1} and (c7;v3)-{-1, +1, +1}
        // (c1;v4)-{+1, -1, -1} and (c2;v5)-{+1, -1, +1}
        // (c5;v6)-{+1, +1, -1} and (c6;v7)-{+1, +1, +1}
        lower_priority_neighbours = {};
        higher_priority_neighbours = {1, 4, 5};
        neighbours = {{0, 3}, {4, 7}, {1, 2}, {5, 6}};
    }
    }
}


template<typename OctreeT, typename DataT>
inline void gather_dual_data(const OctreeT& octree,
                             const typename OctreeT::BlockType* block_ptr,
                             const int scale,
                             const Eigen::Vector3i& primal_corner_coord,
                             DataT data_arr[8],
                             std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f)
{
    const Eigen::Vector3i primal_corner_coord_rel = primal_corner_coord - block_ptr->getCoord();

    std::vector<int> lower_priority_neighbours, higher_priority_neighbours;
    std::vector<std::vector<int>> neighbours;
    norm_dual_corner_idxs(primal_corner_coord_rel, block_ptr->getSize(), lower_priority_neighbours, higher_priority_neighbours, neighbours);

    for (const auto& offset_idx : lower_priority_neighbours) {
        Eigen::Vector3i logical_dual_corner_coord = primal_corner_coord + logical_dual_offset[offset_idx];
        if (!octree.contains(logical_dual_corner_coord)) {
            set_invalid(data_arr[0]);
            return;
        }
        typename OctreeT::BlockType* block_neighbour_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::template block<OctreeT>(logical_dual_corner_coord, octree.getRoot()));
        if (block_neighbour_ptr == nullptr || block_neighbour_ptr->getCurrentScale() <= scale) {
            set_invalid(data_arr[0]);
            return;
        }
    }
    for (const auto& offset_idx : higher_priority_neighbours) {
        Eigen::Vector3i logical_dual_corner_coord = primal_corner_coord + logical_dual_offset[offset_idx];
        if (!octree.contains(logical_dual_corner_coord)) {
            set_invalid(data_arr[0]);
            return;
        }
        typename OctreeT::BlockType* block_neighbour_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::template block<OctreeT>(logical_dual_corner_coord, octree.getRoot()));
        if (block_neighbour_ptr == nullptr || block_neighbour_ptr->getCurrentScale() < scale) {
            set_invalid(data_arr[0]);
            return;
        }
    }

    const int stride = 1 << block_ptr->getCurrentScale();
    for (const auto& offset_idx : neighbours[0]) {
        Eigen::Vector3i logical_dual_corner_coord = primal_corner_coord + logical_dual_offset[offset_idx];
        dual_corner_coords_f[offset_idx] = ((logical_dual_corner_coord / stride) * stride).cast<float>() + stride * se::sample_offset_frac; // TODO:  OctreeT<FieldType>::sample_offset_frac_
        data_arr[offset_idx] = block_ptr->getData(dual_corner_coords_f[offset_idx].cast<int>());                                            /// <- TODO: Should take data from current scale
    }
    for (size_t neighbour_idx = 1; neighbour_idx < neighbours.size(); ++neighbour_idx) {
        Eigen::Vector3i logical_dual_corner_coord = primal_corner_coord + logical_dual_offset[neighbours[neighbour_idx][0]];
        typename OctreeT::BlockType* block_neighbour_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::template block<OctreeT>(logical_dual_corner_coord, octree.getRoot()));
        const int neighbour_stride = 1 << block_neighbour_ptr->getCurrentScale();
        for (const auto& offset_idx : neighbours[neighbour_idx]) {
            logical_dual_corner_coord = primal_corner_coord + logical_dual_offset[offset_idx];
            dual_corner_coords_f[offset_idx] =
                ((logical_dual_corner_coord / neighbour_stride) * neighbour_stride).cast<float>() + neighbour_stride * se::sample_offset_frac; // TODO: OctreeT<FieldType>::sample_offset_frac_
            data_arr[offset_idx] = block_neighbour_ptr->getData(dual_corner_coords_f[offset_idx].cast<int>());                                 /// <- TODO: Should take data from current scale
        }
    }
}


template<typename OctreeT, typename DataT, typename DataToIndexF>
void compute_dual_index(const OctreeT& octree,
                        const typename OctreeT::BlockType* block_ptr,
                        const int scale,
                        const Eigen::Vector3i& primal_corner_coord,
                        uint8_t& edge_pattern_idx,
                        DataT data[8],
                        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& dual_corner_coords_f,
                        DataToIndexF data_to_index)
{
    const unsigned int block_size = block_ptr->getSize();
    // The local case is independent of the scale.
    // lower or upper x boundary (block_coord.x() +0 or +block size) -> (binary) 100 -> local += 4
    // lower or upper y boundary (block_coord.y() +0 or +block size) -> (binary) 010 -> local += 2
    // lower or upper z boundary (block_coord.z() +0 or +block size) -> (binary) 001 -> local += 1
    // local = 0 -> local     - dual contains only local primal neightbours
    // local = 1 -> not local - dual crosses z         boundary
    // local = 2 -> not local - dual crosses y         boundary
    // local = 3 -> not local - dual crosses y + z     boundary
    // local = 4 -> not local - dual crosses x         boundary
    // local = 5 -> not local - dual crosses x + z     boundary
    // local = 6 -> not local - dual crosses x + y     boundary
    // local = 7 -> not local - dual crosses x + y + z boundary
    const unsigned int local = ((primal_corner_coord.x() % block_size == 0) << 2) | ((primal_corner_coord.y() % block_size == 0) << 1) | (primal_corner_coord.z() % block_size == 0);

    if (!local) {
        gather_dual_data(block_ptr, scale, primal_corner_coord.cast<float>(), data, dual_corner_coords_f);
    }
    else {
        gather_dual_data(octree, block_ptr, scale, primal_corner_coord, data, dual_corner_coords_f);
    }

    edge_pattern_idx = data_to_index(data);
}


inline bool checkVertex(const Eigen::Vector3f& vertex_M, const float dim)
{
    return (vertex_M.x() <= 0 || vertex_M.y() <= 0 || vertex_M.z() <= 0 || vertex_M.x() > dim || vertex_M.y() > dim || vertex_M.z() > dim);
}


} // namespace meshing


namespace algorithms {


template<typename OctreeT>
typename OctreeT::MeshType marching_cube_kernel(OctreeT& octree, std::vector<typename OctreeT::BlockType*>& block_ptrs)
{
    typename OctreeT::MeshType triangles;
    using namespace meshing;
    typedef typename OctreeT::BlockType BlockType;

    const int block_size = OctreeT::BlockType::getSize();
    const int octree_size = octree.getSize();

#pragma omp parallel for
    for (size_t block_idx = 0; block_idx < block_ptrs.size(); block_idx++) {
        const BlockType* block_ptr = block_ptrs[block_idx];

        const Eigen::Vector3i& start_coord = block_ptr->getCoord();
        const Eigen::Vector3i last_coord = (start_coord + Eigen::Vector3i::Constant(block_size)).cwiseMin(Eigen::Vector3i::Constant(octree_size - 1));
        for (int x = start_coord.x(); x < last_coord.x(); x++) {
            for (int y = start_coord.y(); y < last_coord.y(); y++) {
                for (int z = start_coord.z(); z < last_coord.z(); z++) {
                    const uint8_t edge_pattern_idx = meshing::compute_index(octree, block_ptr, x, y, z, isosurface::occupied);
                    const int* edges = triTable[edge_pattern_idx];
                    for (unsigned int e = 0; edges[e] != -1 && e < 16; e += 3) {
                        Eigen::Vector3f vertex_0 = interp_vertexes(octree, x, y, z, edges[e]);
                        Eigen::Vector3f vertex_1 = interp_vertexes(octree, x, y, z, edges[e + 1]);
                        Eigen::Vector3f vertex_2 = interp_vertexes(octree, x, y, z, edges[e + 2]);

                        if (checkVertex(vertex_0, octree_size) || checkVertex(vertex_1, octree_size) || checkVertex(vertex_2, octree_size)) {
                            continue;
                        }

                        typename OctreeT::TriangleType temp;
                        temp.vertexes[0] = vertex_0;
                        temp.vertexes[1] = vertex_1;
                        temp.vertexes[2] = vertex_2;
                        if constexpr (OctreeT::TriangleType::colour) {
                            for (size_t i = 0; i < OctreeT::TriangleType::num_vertexes; i++) {
                                temp.vertex_colours[i] = *visitor::getInterp(octree, temp.vertexes[i], [](const auto& x) { return x.rgb; });
                            }
                        }
                        if constexpr (OctreeT::TriangleType::semantics) {
                            temp.class_id = visitor::getData(octree, block_ptr, Eigen::Vector3i(x, y, z)).sem.class_id;
                        }
#pragma omp critical
                        {
                            triangles.push_back(temp);
                        }
                    } // edges

                } // z
            }     // y
        }         // x

    } // block_ptr_itr
    return triangles;
}


template<typename OctreeT>
typename OctreeT::MeshType dual_marching_cube_kernel(OctreeT& octree, std::vector<typename OctreeT::BlockType*>& block_ptrs)
{
    typename OctreeT::MeshType triangles;
    using namespace meshing;
    typedef typename OctreeT::BlockType BlockType;

    const int block_size = OctreeT::BlockType::getSize();
    const int octree_size = octree.getSize();

#pragma omp parallel for
    for (size_t block_idx = 0; block_idx < block_ptrs.size(); block_idx++) {
        const BlockType* block_ptr = block_ptrs[block_idx];
        const int voxel_scale = block_ptr->getCurrentScale();
        const int voxel_stride = 1 << voxel_scale;
        const Eigen::Vector3i& start_coord = block_ptr->getCoord();
        const Eigen::Vector3i last_coord = (start_coord + Eigen::Vector3i::Constant(block_size)).cwiseMin(Eigen::Vector3i::Constant(octree_size - 1));
        for (int x = start_coord.x(); x <= last_coord.x(); x += voxel_stride) {
            for (int y = start_coord.y(); y <= last_coord.y(); y += voxel_stride) {
                for (int z = start_coord.z(); z <= last_coord.z(); z += voxel_stride) {
                    const Eigen::Vector3i primal_corner_coord = Eigen::Vector3i(x, y, z);

                    if (x == last_coord.x() || y == last_coord.y() || z == last_coord.z()) {
                        if (!se::fetcher::template block<OctreeT>(primal_corner_coord, octree.getRoot())) {
                            continue;
                        }
                    }

                    uint8_t edge_pattern_idx;
                    typename OctreeT::DataType data[8];
                    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> dual_corner_coords_f(8, Eigen::Vector3f::Constant(0));
                    meshing::compute_dual_index(octree, block_ptr, voxel_scale, primal_corner_coord, edge_pattern_idx, data, dual_corner_coords_f, isosurface::occupied);
                    const int* edges = triTable[edge_pattern_idx];
                    for (unsigned int e = 0; edges[e] != -1 && e < 16; e += 3) {
                        Eigen::Vector3f vertex_0 = interp_dual_vertexes(edges[e], data, dual_corner_coords_f);
                        Eigen::Vector3f vertex_1 = interp_dual_vertexes(edges[e + 1], data, dual_corner_coords_f);
                        Eigen::Vector3f vertex_2 = interp_dual_vertexes(edges[e + 2], data, dual_corner_coords_f);

                        if (checkVertex(vertex_0, octree_size) || checkVertex(vertex_1, octree_size) || checkVertex(vertex_2, octree_size)) {
                            continue;
                        }
                        typename OctreeT::TriangleType temp;
                        temp.vertexes[0] = vertex_0;
                        temp.vertexes[1] = vertex_1;
                        temp.vertexes[2] = vertex_2;
                        temp.max_vertex_scale = voxel_scale;
                        if constexpr (OctreeT::TriangleType::colour) {
                            for (size_t i = 0; i < OctreeT::TriangleType::num_vertexes; i++) {
                                temp.vertex_colours[i] = *visitor::getInterp(octree, temp.vertexes[i], [](const auto& x) { return x.rgb; });
                            }
                        }
                        if constexpr (OctreeT::TriangleType::semantics) {
                            int _;
                            temp.class_id = visitor::getData(octree, block_ptr, Eigen::Vector3i(x, y, z), voxel_scale, _).sem.class_id;
                        }
#pragma omp critical
                        {
                            triangles.push_back(temp);
                        };
                    } // edges

                } // z
            }     // y
        }         // x

    } // block_ptr_itr
    return triangles;
}


template<typename OctreeT>
typename OctreeT::MeshType marching_cube(OctreeT& octree)
{
    TICK("primal-marching-cube")
    typedef typename OctreeT::BlockType BlockType;

    TICK("marching-cube-create-block-list")
    std::vector<BlockType*> block_ptrs;
    for (auto block_ptr_itr = se::BlocksIterator<OctreeT>(&octree); block_ptr_itr != se::BlocksIterator<OctreeT>(); ++block_ptr_itr) {
        block_ptrs.push_back(static_cast<BlockType*>(*block_ptr_itr));
    }
    TOCK("marching-cube-create-block-list")

    const typename OctreeT::MeshType triangles = se::algorithms::marching_cube_kernel(octree, block_ptrs);

    TOCK("primal-marching-cube")
    return triangles;
}


template<typename OctreeT>
typename OctreeT::MeshType marching_cube(OctreeT& octree, const int time_stamp)
{
    TICK("primal-marching-cube")
    typedef typename OctreeT::BlockType BlockType;

    std::vector<BlockType*> block_ptrs;
    for (auto block_ptr_itr = se::UpdateIterator<OctreeT>(&octree, std::max(((int) octree.getRoot()->getTimeStamp()) - time_stamp, 0)); block_ptr_itr != se::UpdateIterator<OctreeT>();
         ++block_ptr_itr) {
        block_ptrs.push_back(static_cast<BlockType*>(*block_ptr_itr));
    }

    const typename OctreeT::MeshType triangles = se::algorithms::marching_cube_kernel(octree, block_ptrs);

    TOCK("primal-marching-cube")
    return triangles;
}


template<typename OctreeT>
typename OctreeT::MeshType dual_marching_cube(OctreeT& octree)
{
    TICK("dual-marching-cube")
    typedef typename OctreeT::BlockType BlockType;

    std::vector<BlockType*> block_ptrs;
    for (auto block_ptr_itr = se::BlocksIterator<OctreeT>(&octree); block_ptr_itr != se::BlocksIterator<OctreeT>(); ++block_ptr_itr) {
        block_ptrs.push_back(static_cast<BlockType*>(*block_ptr_itr));
    }

    const typename OctreeT::MeshType triangles = se::algorithms::dual_marching_cube_kernel(octree, block_ptrs);

    TOCK("dual-marching-cube")
    return triangles;
}


template<typename OctreeT>
typename OctreeT::MeshType dual_marching_cube(OctreeT& octree, const int time_stamp)
{
    TICK("dual-marching-cube")
    typedef typename OctreeT::BlockType BlockType;

    std::vector<BlockType*> block_ptrs;
    for (auto block_ptr_itr = se::UpdateIterator<OctreeT>(&octree, std::max((octree.getRoot()->getTimeStamp()) - time_stamp, 0)); block_ptr_itr != se::UpdateIterator<OctreeT>(); ++block_ptr_itr) {
        block_ptrs.push_back(static_cast<BlockType*>(*block_ptr_itr));
    }

    const typename OctreeT::MeshType triangles = se::algorithms::dual_marching_cube_kernel(octree, block_ptrs);

    TOCK("dual-marching-cube")
    return triangles;
}


} // namespace algorithms
} // namespace se

#endif // SE_MARCHING_CUBE_IMPL_HPP
