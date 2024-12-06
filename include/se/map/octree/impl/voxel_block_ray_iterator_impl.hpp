/*
 * SPDX-FileCopyrightText: 2009-2011 NVIDIA Corporation
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_VOXEL_BLOCK_RAY_ITERATOR_IMPL_HPP
#define SE_VOXEL_BLOCK_RAY_ITERATOR_IMPL_HPP


namespace se {


template<typename MapT>
VoxelBlockRayIterator<MapT>::VoxelBlockRayIterator(const MapT& map, const Eigen::Vector3f& ray_origin_M, const Eigen::Vector3f& ray_dir_M, const float near_plane, const float far_plane) :
        map_(map), octree_(*(map_.getOctree())), scaling_((octree_.getSize() * map_.getRes()))
{
    pos_ = Eigen::Vector3f::Ones();
    idx_ = 0;
    parent_ptr_ = static_cast<NodeType*>(octree_.getRoot());
    child_ptr_ = nullptr;
    scale_exp2_ = 0.5f;
    scale_ = CAST_STACK_DEPTH - 1;
    min_scale_ = CAST_STACK_DEPTH - log2(octree_.getSize() / BlockType::size);
    state_ = INIT;

    // Zero-initialize the stack
    memset(stack_, 0, CAST_STACK_DEPTH * sizeof(StackEntry));

    // Ensure all elements of ray_dir_M_ are non-zero.
    const float epsilon = exp2f(-log2(octree_.getSize()));
    ray_dir_M_.x() = fabsf(ray_dir_M.x()) < epsilon ? copysignf(epsilon, ray_dir_M.x()) : ray_dir_M.x();
    ray_dir_M_.y() = fabsf(ray_dir_M.y()) < epsilon ? copysignf(epsilon, ray_dir_M.y()) : ray_dir_M.y();
    ray_dir_M_.z() = fabsf(ray_dir_M.z()) < epsilon ? copysignf(epsilon, ray_dir_M.z()) : ray_dir_M.z();

    // Scale the origin to be in the interval [1, 2].
    Eigen::Vector3f ray_origin;
    map_.template pointToVoxel<Safe::Off>(ray_origin_M, ray_origin);
    ray_origin = ray_origin / octree_.getSize() + Eigen::Vector3f::Ones();

    // Precompute the ray coefficients.
    t_coef_ = -1.f * ray_dir_M_.cwiseAbs().cwiseInverse();
    t_bias_ = t_coef_.cwiseProduct(ray_origin);


    // Build the octant mask to to mirror the coordinate system such that
    // each ray component points in negative coordinates. The octree is
    // assumed to reside at coordinates [1, 2]
    octant_mask_ = 7;
    if (ray_dir_M_.x() > 0.0f) {
        octant_mask_ ^= 1;
        t_bias_.x() = 3.0f * t_coef_.x() - t_bias_.x();
    }
    if (ray_dir_M_.y() > 0.0f) {
        octant_mask_ ^= 2;
        t_bias_.y() = 3.0f * t_coef_.y() - t_bias_.y();
    }
    if (ray_dir_M_.z() > 0.0f) {
        octant_mask_ ^= 4;
        t_bias_.z() = 3.0f * t_coef_.z() - t_bias_.z();
    }

    // Find the min-max t ranges.
    t_min_init_ = (2.0f * t_coef_ - t_bias_).maxCoeff();
    t_max_init_ = (t_coef_ - t_bias_).minCoeff();
    h_ = t_max_init_;
    t_min_init_ = std::max(t_min_init_, near_plane / scaling_); // TODO: [x]
    t_max_init_ = std::min(t_max_init_, far_plane / scaling_);  // TODO: [x]
    t_min_ = t_min_init_;
    t_max_ = t_max_init_;

    // Initialise the ray position.
    if (1.5f * t_coef_.x() - t_bias_.x() > t_min_) {
        idx_ ^= 1;
        pos_.x() = 1.5f;
    }
    if (1.5f * t_coef_.y() - t_bias_.y() > t_min_) {
        idx_ ^= 2;
        pos_.y() = 1.5f;
    }
    if (1.5f * t_coef_.z() - t_bias_.z() > t_min_) {
        idx_ ^= 4;
        pos_.z() = 1.5f;
    }
}


template<typename MapT>
typename VoxelBlockRayIterator<MapT>::BlockType* VoxelBlockRayIterator<MapT>::next()
{
    if (state_ == ADVANCE) {
        advance_ray();
    }
    else if (state_ == FINISHED) {
        return nullptr;
    }

    while (scale_ < CAST_STACK_DEPTH) {
        t_corner_ = pos_.cwiseProduct(t_coef_) - t_bias_;
        tc_max_ = t_corner_.minCoeff();

        child_ptr_ = parent_ptr_->getChild(idx_ ^ octant_mask_ ^ 7);

        if (scale_ == min_scale_ && child_ptr_ != nullptr) {
            state_ = ADVANCE;
            return static_cast<BlockType*>(child_ptr_);
        }
        else if (child_ptr_ != nullptr && t_min_ <= t_max_) {
            // If the child is valid, descend the tree hierarchy.
            descend();
            continue;
        }
        advance_ray();
    }
    state_ = FINISHED;
    return nullptr;
}


template<typename MapT>
float VoxelBlockRayIterator<MapT>::tmin() const
{
    return t_min_init_ * scaling_;
}


template<typename MapT>
float VoxelBlockRayIterator<MapT>::tmax() const
{
    return t_max_init_ * scaling_;
}


template<typename MapT>
float VoxelBlockRayIterator<MapT>::tcmin() const
{
    return t_min_ * scaling_;
}


template<typename MapT>
float VoxelBlockRayIterator<MapT>::tcmax() const
{
    return tc_max_ * scaling_;
}


template<typename MapT>
inline void VoxelBlockRayIterator<MapT>::advance_ray()
{
    const int step_mask = (t_corner_.x() <= tc_max_) | ((t_corner_.y() <= tc_max_) << 1) | ((t_corner_.z() <= tc_max_) << 2);
    pos_.x() -= scale_exp2_ * bool(step_mask & 1);
    pos_.y() -= scale_exp2_ * bool(step_mask & 2);
    pos_.z() -= scale_exp2_ * bool(step_mask & 4);

    t_min_ = tc_max_;
    idx_ ^= step_mask;

    // POP if bits flips disagree with ray direction
    if ((idx_ & step_mask) != 0) {
        // Get the different bits for each component. This is done by xoring
        // the bit patterns of the new and old pos. This works because the
        // volume has been scaled between [1, 2]. Still digging why this is the
        // case.
        unsigned int differing_bits = 0;
        if ((step_mask & 1) != 0) {
            differing_bits |= floatAsInt(pos_.x()) ^ floatAsInt(pos_.x() + scale_exp2_);
        }
        if ((step_mask & 2) != 0) {
            differing_bits |= floatAsInt(pos_.y()) ^ floatAsInt(pos_.y() + scale_exp2_);
        }
        if ((step_mask & 4) != 0) {
            differing_bits |= floatAsInt(pos_.z()) ^ floatAsInt(pos_.z() + scale_exp2_);
        }

        // Get the scale at which the two differs. Here's there are different
        // subtlelties related to how fp are stored.
        // MIND BLOWN: differing bit (i.e. the MSB) extracted using the
        // exponent part of the fp representation.
        scale_ = (floatAsInt((float) differing_bits) >> 23) - 127;         // position of the highest bit
        scale_exp2_ = intAsFloat((scale_ - CAST_STACK_DEPTH + 127) << 23); // exp2f(scale - s_max)
        const StackEntry& e = stack_[scale_];
        parent_ptr_ = e.parent;
        t_max_ = e.t_max;

        // Round cube position and extract child slot index.
        const int shx = floatAsInt(pos_.x()) >> scale_;
        const int shy = floatAsInt(pos_.y()) >> scale_;
        const int shz = floatAsInt(pos_.z()) >> scale_;
        pos_.x() = intAsFloat(shx << scale_);
        pos_.y() = intAsFloat(shy << scale_);
        pos_.z() = intAsFloat(shz << scale_);
        idx_ = (shx & 1) | ((shy & 1) << 1) | ((shz & 1) << 2);

        h_ = 0.0f;
        child_ptr_ = nullptr;
    }
}


template<typename MapT>
inline void VoxelBlockRayIterator<MapT>::descend()
{
    const float tv_max = std::min(t_max_, tc_max_);
    const float half = scale_exp2_ * 0.5f;
    const Eigen::Vector3f t_centre = half * t_coef_ + t_corner_;

    // Descend to the first child if the resulting t-span is non-empty.
    if (tc_max_ < h_) {
        stack_[scale_] = {scale_, parent_ptr_, t_max_};
    }

    h_ = tc_max_;
    parent_ptr_ = static_cast<NodeType*>(child_ptr_);

    idx_ = 0;
    scale_--;
    scale_exp2_ = half;
    idx_ ^= (t_centre.x() > t_min_) ? 1 : 0;
    idx_ ^= (t_centre.y() > t_min_) ? 2 : 0;
    idx_ ^= (t_centre.z() > t_min_) ? 4 : 0;

    pos_.x() += scale_exp2_ * bool(idx_ & 1);
    pos_.y() += scale_exp2_ * bool(idx_ & 2);
    pos_.z() += scale_exp2_ * bool(idx_ & 4);

    t_max_ = tv_max;
    child_ptr_ = nullptr;
}


} // namespace se


#endif // SE_VOXEL_BLOCK_RAY_ITERATOR_IMPL_HPP
