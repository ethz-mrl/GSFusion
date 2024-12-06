/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ITERATOR_HPP
#define SE_ITERATOR_HPP

#include <stack>

#include "octree.hpp"
#include "se/map/utils/octant_util.hpp"


namespace se {

template<typename DerivedT>
struct BaseTraits;

/** \brief Iterates over all valid data in the octree at the last scale it
 * was updated at.
 * The iterator performs a depth-first traversal of the octree. To use it
 * just use the se::Octree::begin() and se::Octree::end() functions or a
 * range-based for loop:
 *
 * ``` cpp
 * for (auto& volume : octree) {
 *     // Do stuff with volume
 * }
 * ```
 *
 * \note Changes to the se::Octree while iterating will result in strange
 * behavior.
 */
template<typename DerivedT>
class BaseIterator {
    public:
    typedef typename BaseTraits<DerivedT>::OctreeType OctreeType;
    typedef typename BaseTraits<DerivedT>::NodeType NodeType;
    typedef typename BaseTraits<DerivedT>::BlockType BlockType;

    BaseIterator();

    BaseIterator(OctreeType* octree_ptr);

    BaseIterator(const BaseIterator& other);

    BaseIterator& operator++();

    BaseIterator operator++(int);

    bool operator==(const BaseIterator& other) const;

    bool operator!=(const BaseIterator& other) const;

    se::OctantBase* operator*() const;

    // Iterator traits
    using difference_type = long;
    using value_type = se::OctantBase;
    using pointer = se::OctantBase*;
    using reference = se::OctantBase&;
    using iterator_category = std::forward_iterator_tag;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    void init();

    private:
    DerivedT& underlying()
    {
        return static_cast<DerivedT&>(*this);
    }
    const DerivedT& underlying() const
    {
        return static_cast<const DerivedT&>(*this);
    }

    // Find the next Volume with valid data.
    void nextData();

    // Reset the iterator state to invalid. Used when finished iterating.
    void clear();

    // The 3 stacks should always be kept in sync.
    // Pointers to the Nodes that haven't been checked yet.
    std::stack<se::OctantBase*> octant_stack_;

    se::OctantBase* octant_;

    OctreeType* octree_ptr_ = nullptr;
};


template<typename OctreeT>
class OctreeIterator : public BaseIterator<OctreeIterator<OctreeT>> {
    public:
    OctreeIterator() : BaseIterator<OctreeIterator<OctreeT>>(){};

    OctreeIterator(OctreeT* octree_ptr) : BaseIterator<OctreeIterator<OctreeT>>(octree_ptr)
    {
        this->init();
    };

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* /* octant_ptr */)
    {
        return true;
    }

    protected:
    friend class BaseIterator<OctreeIterator<OctreeT>>;
};


template<typename OctreeT>
class NodesIterator : public BaseIterator<NodesIterator<OctreeT>> {
    public:
    NodesIterator() : BaseIterator<NodesIterator<OctreeT>>(){};

    NodesIterator(OctreeT* octree_ptr) : BaseIterator<NodesIterator<OctreeT>>(octree_ptr)
    {
        this->init();
    };

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* octant_ptr)
    {
        return !octant_ptr->isBlock();
    }

    protected:
    friend class BaseIterator<NodesIterator<OctreeT>>;
};


template<typename OctreeT>
class BlocksIterator : public BaseIterator<BlocksIterator<OctreeT>> {
    public:
    BlocksIterator() : BaseIterator<BlocksIterator<OctreeT>>(){};

    BlocksIterator(OctreeT* octree_ptr) : BaseIterator<BlocksIterator<OctreeT>>(octree_ptr)
    {
        this->init();
    };

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* octant_ptr)
    {
        return octant_ptr->isBlock();
    }

    protected:
    friend class BaseIterator<BlocksIterator<OctreeT>>;
};


template<typename OctreeT>
class LeavesIterator : public BaseIterator<LeavesIterator<OctreeT>> {
    public:
    LeavesIterator() : BaseIterator<LeavesIterator<OctreeT>>(){};

    LeavesIterator(OctreeT* octree_ptr) : BaseIterator<LeavesIterator<OctreeT>>(octree_ptr)
    {
        this->init();
    };

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* octant_ptr)
    {
        return octant_ptr->isLeaf();
    }

    protected:
    friend class BaseIterator<LeavesIterator<OctreeT>>;
};


template<typename OctreeT>
class UpdateIterator : public BaseIterator<UpdateIterator<OctreeT>> {
    public:
    UpdateIterator() : BaseIterator<UpdateIterator<OctreeT>>(), time_stamp_(0){};

    UpdateIterator(OctreeT* octree_ptr, timestamp_t time_stamp) : BaseIterator<UpdateIterator<OctreeT>>(octree_ptr), time_stamp_(time_stamp)
    {
        this->init();
    };

    static constexpr bool has_ignore_condition = true;

    bool isNext(se::OctantBase* octant_ptr)
    {
        return octant_ptr->isBlock() && octant_ptr->getTimeStamp() >= time_stamp_;
    }

    bool doIgnore(se::OctantBase* octant_ptr)
    {
        return octant_ptr->getTimeStamp() < time_stamp_;
    }

    const timestamp_t time_stamp_;

    protected:
    friend class BaseIterator<UpdateIterator<OctreeT>>;
};


template<typename MapT, typename SensorT>
class FrustumIterator : public BaseIterator<FrustumIterator<MapT, SensorT>> {
    public:
    typedef typename MapT::OctreeType OctreeType;
    typedef typename MapT::OctreeType::NodeType NodeType;
    typedef typename MapT::OctreeType::BlockType BlockType;


    FrustumIterator() : BaseIterator<FrustumIterator<MapT, SensorT>>(){};

    FrustumIterator(MapT& map, const SensorT& sensor, const Eigen::Matrix4f& T_SM) :
            BaseIterator<FrustumIterator<MapT, SensorT>>(map.getOctree().get()), map_ptr_(&map), sensor_ptr_(&sensor), T_SM_(T_SM)
    {
        this->init();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    bool isNext(se::OctantBase* octant_ptr)
    {
        return octant_ptr->isBlock();
    }

    bool doIgnore(se::OctantBase* octant_ptr)
    {
        Eigen::Vector3f octant_centre_point_M;
        const int octant_size = se::octantops::octant_to_size<typename MapT::OctreeType>(octant_ptr);
        map_ptr_->voxelToPoint(octant_ptr->getCoord(), octant_size, octant_centre_point_M);
        // Convert it to the sensor frame.
        const Eigen::Vector3f octant_centre_point_S = (T_SM_ * octant_centre_point_M.homogeneous()).template head<3>();

        float octant_radius = std::sqrt(3.0f) / 2.0f * map_ptr_->getRes() * octant_size;
        bool do_ignore = !sensor_ptr_->sphereInFrustum(octant_centre_point_S, octant_radius);
        return do_ignore;
    }

    static constexpr bool has_ignore_condition = true;

    MapT* map_ptr_;
    const SensorT* sensor_ptr_;
    const Eigen::Matrix4f T_SM_; // TODO: Needs to be ref?

    friend class BaseIterator<FrustumIterator<MapT, SensorT>>;
};


// Declare and define a base_traits specialization for the octree iterator:
template<typename OctreeT>
struct BaseTraits<OctreeIterator<OctreeT>> {
    typedef OctreeT OctreeType;
    typedef typename OctreeT::NodeType NodeType;
    typedef typename OctreeT::BlockType BlockType;
};


// Declare and define a base_traits specialization for the nodes iterator:
template<typename OctreeT>
struct BaseTraits<NodesIterator<OctreeT>> {
    typedef OctreeT OctreeType;
    typedef typename OctreeT::NodeType NodeType;
    typedef typename OctreeT::BlockType BlockType;
};


// Declare and define a base_traits specialization for the block iterator:
template<typename OctreeT>
struct BaseTraits<BlocksIterator<OctreeT>> {
    typedef OctreeT OctreeType;
    typedef typename OctreeT::NodeType NodeType;
    typedef typename OctreeT::BlockType BlockType;
};


// Declare and define a base_traits specialization for the leaves iterator:
template<typename OctreeT>
struct BaseTraits<LeavesIterator<OctreeT>> {
    typedef OctreeT OctreeType;
    typedef typename OctreeT::NodeType NodeType;
    typedef typename OctreeT::BlockType BlockType;
};


// Declare and define a base_traits specialization for the update iterator:
template<typename OctreeT>
struct BaseTraits<UpdateIterator<OctreeT>> {
    typedef OctreeT OctreeType;
    typedef typename OctreeT::NodeType NodeType;
    typedef typename OctreeT::BlockType BlockType;
};

// Declare and define a base_traits specialization for the update iterator:
template<typename MapT, typename SensorT>
struct BaseTraits<FrustumIterator<MapT, SensorT>> {
    typedef typename MapT::OctreeType OctreeType;
    typedef typename MapT::OctreeType::NodeType NodeType;
    typedef typename MapT::OctreeType::BlockType BlockType;
};


} // namespace se

#include "impl/iterator_impl.hpp"

#endif // SE_ITERATOR_HPP
