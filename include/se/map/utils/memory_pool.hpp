/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MEMORY_POOL_HPP
#define SE_MEMORY_POOL_HPP

#include <Eigen/Core>
#include <boost/pool/pool.hpp>

namespace se {

/** Manages memory for octree nodes and blocks in an efficient manner. Never deallocate nodes/blocks
 * created through se::MemoryPool using \p delete.
 */
template<typename NodeT, typename BlockT>
class MemoryPool {
    public:
    typedef typename NodeT::DataType DataType;

    MemoryPool() : node_buffer_(sizeof(NodeT)), block_buffer_(sizeof(BlockT))
    {
    }

    /** Allocate the root node with coordinates in voxels \p coord and edge length in voxels \p
     * size.
     */
    NodeT* allocateRoot(const Eigen::Vector3i& coord, const int size)
    {
        return new (node_buffer_.malloc()) NodeT(coord, size, DataType());
    }

    /** Allocate a node.
     *
     * \param[in] parent_ptr The pointer to the parent of the node to be allocated.
     * \param[in] child_idx  The child index of the node to be allocated.
     * \param[in] init_data  The data to initialise the node with.
     *
     * \return The pointer to the allocated node.
     */
    NodeT* allocateNode(NodeT* parent_ptr, const int child_idx, const DataType& init_data)
    {
        return new (node_buffer_.malloc()) NodeT(parent_ptr, child_idx, init_data);
    }

    /** Allocate a block.
     *
     * \param[in] parent_ptr The pointer to the parent of the block to be allocated.
     * \param[in] child_idx  The child index of the block to be allocated.
     * \param[in] init_data  The data to initialise the block with.
     *
     * \return The pointer to the allocated block.
     */
    BlockT* allocateBlock(NodeT* parent_ptr, const int child_idx, const DataType& init_data)
    {
        return new (block_buffer_.malloc()) BlockT(parent_ptr, child_idx, init_data);
    }

    /** Destruct and deallocate the node pointed to by \p node_ptr.
     */
    void deleteNode(NodeT* node_ptr)
    {
        node_ptr->~NodeT();
        node_buffer_.free(node_ptr);
    }

    /** Destruct and deallocate the block pointed to by \p block_ptr.
     */
    void deleteBlock(BlockT* block_ptr)
    {
        block_ptr->~BlockT();
        block_buffer_.free(block_ptr);
    }

    boost::pool<> node_buffer_;
    boost::pool<> block_buffer_;
};

} // namespace se

#endif // SE_MEMORY_POOL_HPP
