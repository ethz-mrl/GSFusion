/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_OCTREE_HPP
#define SE_OCTREE_HPP

#include <Eigen/Geometry>
#include <memory>

#include "se/map/algorithms/mesh.hpp"
#include "se/map/octant/octant.hpp"
#include "se/map/octree/iterator.hpp"
#include "se/map/utils/key_util.hpp"
#include "se/map/utils/memory_pool.hpp"
#include "se/map/utils/setup_util.hpp"

namespace se {

/** The octree data structure containing the map data. It is the memory manager of the map since it
 * is the only entity that is able to allocate and deallocate nodes and blocks. There are
 * specialized functions and classes for accessing (se::visitor) and modifying (se::MapIntegrator)
 * the map data as this isn't done through this class. At the maximum octree depth data is stored in
 * blocks of \p BlockSize<sup>3</sup> voxels.
 *
 * https://en.wikipedia.org/wiki/Octree
 *
 * \tparam DataT     The data type stored in the octree.
 * \tparam ResT      se::Res::Single if data is only stored in the octree leaves or se::Res::Multi
 *                   if data is stored in all octree nodes.
 * \tparam BlockSize The edge length of a voxel block in voxels. It must be a power of two.
 */
template<typename DataT, Res ResT = Res::Single, int BlockSize = 8>
class Octree {
    public:
    typedef std::shared_ptr<Octree<DataT, ResT, BlockSize>> Ptr;
    typedef DataT DataType;
    typedef Node<DataT, ResT> NodeType;
    typedef Block<DataT, ResT, BlockSize> BlockType;

    typedef MeshFace<3, DataT::col_, DataT::sem_> TriangleType;
    typedef MeshFace<4, DataT::col_, DataT::sem_> QuadType;
    typedef Mesh<TriangleType> MeshType;
    typedef Mesh<QuadType> QuadMeshType;

    /** Initialize an octree with an edge length of at least \p size voxels. The actual edge length
     * in voxels will be the smallest power of 2 that is greater or equal to \p size. and at least
     * 2 * \p BlockSize.
     */
    Octree(const int size);

    /** The copy constructor is explicitly deleted.
     */
    Octree(const Octree&) = delete;

    /** The copy assignment operator is explicitly deleted.
     */
    Octree& operator=(const Octree&) = delete;

    OctreeIterator<Octree<DataT, ResT, BlockSize>> begin();
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> begin() const;
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> cbegin() const;
    OctreeIterator<Octree<DataT, ResT, BlockSize>> end();
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> end() const;
    OctreeIterator<const Octree<DataT, ResT, BlockSize>> cend() const;

    /** Test if point \p voxel_coord with coordinates in voxels is contained in the octree.
     */
    bool contains(const Eigen::Vector3i& voxel_coord) const;

    /** Get the pointer octree's root octant.
     */
    OctantBase* getRoot()
    {
        return root_ptr_;
    };

    /** Get the pointer octree's root octant.
     *
     * \todo Return `const OctantBase*` once all relevant functions have been made const-correct.
     */
    OctantBase* getRoot() const
    {
        return root_ptr_;
    };

    /** Get the length of the octree edge in voxels.
     */
    int getSize() const
    {
        return size_;
    }

    /**
     * Get the maximum scale of the octree, i.e. the scale of the root node.
     */
    int getMaxScale() const
    {
        return math::log2_const(size_);
    }

    /** Get the depth voxel blocks are allocated at.
     */
    int getBlockDepth() const
    {
        return math::log2_const(size_) - math::log2_const(BlockSize);
    }

    /** Allocate a child of a node.
     *
     * \note The returned pointer is of type se::OctantBase as the child might be a node or block.
     *
     * \warning This function might be dangerous when using Multires Occupancy. Use
     * se::Octree::allocateChildren() if unsure.
     *
     * \param[in]  parent_ptr The parent of the octant to be allocated.
     * \param[in]  child_idx  The child index of the octant to be allocated.
     * \param[out] child_ptr  The pointer to the allocated or fetched octant.
     *
     * \return True if the child was allocated, false if it was already allocated.
     */
    bool allocate(NodeType* parent_ptr, const int child_idx, OctantBase*& child_ptr);

    /** Allocate all the child nodes of \p parent_ptr.
     */
    void allocateChildren(NodeType* parent_ptr);

    /** Recursively delete all children of \p parent_ptr.
     */
    void deleteChildren(NodeType* parent_ptr);

    /** Return the axis-aligned bounding box of the octree's allocated leaves. The bounding box
     * contains the whole allocated volume, not just the voxel origins thus the coordinates of its
     * vertices can be in the interval [0, size_] inclusive.
     */
    const Eigen::AlignedBox3i& aabb() const;

    /** Extend the octree allocated leaf AABB to contain the octant with coordinates in voxels \p
     * voxel_coord and edge length in voxels \p size.
     *
     * \note This is typically only needed to update the AABB with leaf nodes as they can't
     * efficiently be detected during allocation since all nodes are leaves when allocated. This
     * function should be called only for newly allocated leaf nodes from se::VolumeCarver.
     */
    void aabbExtend(const Eigen::Vector3i& voxel_coord, const int size);

    static constexpr Field fld_ = DataT::fld_;
    static constexpr Colour col_ = DataT::col_;
    static constexpr Semantics sem_ = DataT::sem_;
    static constexpr Res res_ = ResT;
    /** The edge length of a voxel block in voxels. */
    static constexpr int block_size = BlockSize;
    /** The maximum scale of a voxel block. */
    static constexpr scale_t max_block_scale = math::log2_const(BlockSize);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    const int size_;
    // Allocates and deallocates memory for nodes and blocks.
    MemoryPool<NodeType, BlockType> memory_pool_;
    OctantBase* const root_ptr_; // The pointer lifetime is managed by memory_pool_.
    Eigen::AlignedBox3i aabb_;

    static_assert(math::is_power_of_two(BlockSize));
};


template<typename DataT, Res ResT, int BlockSize>
constexpr scale_t Octree<DataT, ResT, BlockSize>::max_block_scale;

} // namespace se

#include "impl/octree_impl.hpp"

#endif // SE_OCTREE_HPP
