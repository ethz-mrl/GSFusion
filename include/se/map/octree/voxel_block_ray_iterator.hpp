/*
 * SPDX-FileCopyrightText: 2009-2011 NVIDIA Corporation
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VOXEL_BLOCK_RAY_ITERATOR_HPP
#define SE_VOXEL_BLOCK_RAY_ITERATOR_HPP

#include "se/map/octree/octree.hpp"

#define CAST_STACK_DEPTH 23

namespace se {


template<typename MapT>
class VoxelBlockRayIterator {
    typedef typename MapT::OctreeType::NodeType NodeType;
    typedef typename MapT::OctreeType::BlockType BlockType;

    public:
    VoxelBlockRayIterator(const MapT& map, const Eigen::Vector3f& ray_origin_M, const Eigen::Vector3f& ray_dir_M, const float near_plane, const float far_plane);

    /** \brief Return a pointer to the next se::VoxelBlock along the ray.
   *
   * \return A pointer to an se::VoxelBlock or nullptr if all se::VoxelBlock
   * along the ray have been iterated through.
   */
    BlockType* next();

    /**
   * \brief The distance along the ray until the octree is entered.
   *
   * \return The distance in meters.
   */
    float tmin() const;

    /**
   * \brief The distance along the ray until the octree is exited.
   *
   * \return The distance in meters.
   */
    float tmax() const;

    /**
   * \brief The distance along the ray until the current se::VoxelBlock is
   * entered.
   *
   * \note Returns the same value as se::VoxelBlockRayIterator::tmin() if
   * se::VoxelBlockRayIterator::next() has not been called yet.
   *
   * \return The distance in meters.
   */
    float tcmin() const;

    /**
   * \brief The distance along the ray until the current se::VoxelBlock is
   * exited.
   *
   * \warning Returns an undefined value if se::VoxelBlockRayIterator::next()
   * has not been called yet.
   *
   * \return The distance in meters.
   */
    float tcmax() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    struct StackEntry {
        int scale;
        NodeType* parent;
        float t_max;
    };


    enum STATE { INIT, ADVANCE, FINISHED };


    MapT& map_;
    const typename MapT::OctreeType& octree_;
    Eigen::Vector3f ray_origin_M_;
    Eigen::Vector3f ray_dir_M_;
    Eigen::Vector3f pos_;
    Eigen::Vector3f t_coef_;
    Eigen::Vector3f t_bias_;
    Eigen::Vector3f t_corner_;
    StackEntry stack_[CAST_STACK_DEPTH];
    NodeType* parent_ptr_;
    se::OctantBase* child_ptr_;
    int idx_;
    int scale_;
    float scaling_;
    int min_scale_;
    int octant_mask_;
    float scale_exp2_;
    float t_min_;
    float t_min_init_;
    float t_max_;
    float t_max_init_;
    float tc_max_;
    float h_;
    STATE state_;


    /**
   * \brief Reinterpret the binary representation of a float as an int.
   */
    static inline int floatAsInt(const float value)
    {
        union float_as_int {
            float f;
            int i;
        };

        float_as_int u;
        u.f = value;
        return u.i;
    }


    /**
   * \brief Reinterpret the binary representation of an int as a float.
   */
    static inline float intAsFloat(const int value)
    {
        union int_as_float {
            int i;
            float f;
        };

        int_as_float u;
        u.i = value;
        return u.f;
    }

    /** \brief Advance the ray.
   */
    inline void advance_ray();

    /**
   * \brief Descend the hiararchy and compute the next child position.
   */
    inline void descend();
};


} // namespace se


#include "impl/voxel_block_ray_iterator_impl.hpp"


#endif // SE_VOXEL_BLOCK_RAY_ITERATOR_HPP
