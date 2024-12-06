/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_NODE_IMPL_HPP
#define SE_NODE_IMPL_HPP

namespace se {

template<typename NodeT>
int get_child_idx(const Eigen::Vector3i& child_coord, const NodeT* parent_ptr)
{
    const Eigen::Vector3i parent_coord = parent_ptr->getCoord();
    const int parent_size = parent_ptr->getSize();

    assert(keyops::is_child(keyops::encode_key(parent_coord, math::log2_const(parent_size)), keyops::encode_key(child_coord, 0)));

    Eigen::Vector3i offset = child_coord - parent_coord;
    const unsigned int child_size = parent_size >> 1;

    return ((offset.x() & child_size) > 0) + 2 * ((offset.y() & child_size) > 0) + 4 * ((offset.z() & child_size) > 0);
}


template<typename DataT, Res ResT>
Node<DataT, ResT>::Node(const Eigen::Vector3i& coord, const int size, const DataT& init_data) :
        OctantBase(false, coord, nullptr), std::conditional<ResT == Res::Single, NodeSingleRes<DataT>, NodeMultiRes<DataT, Node<DataT, ResT>>>::type(init_data), size_(size)
{
    children_ptr_.fill(nullptr);
}


template<typename DataT, Res ResT>
Node<DataT, ResT>::Node(Node* parent_ptr, const int child_idx, const DataT& init_data) :
        OctantBase(false, parent_ptr->coord_ + (parent_ptr->size_ >> 1) * Eigen::Vector3i((1 & child_idx) > 0, (2 & child_idx) > 0, (4 & child_idx) > 0), parent_ptr),
        std::conditional<ResT == Res::Single, NodeSingleRes<DataT>, NodeMultiRes<DataT, Node<DataT, ResT>>>::type(init_data),
        size_(parent_ptr->size_ >> 1)
{
    children_ptr_.fill(nullptr);
}


template<typename DataT, Res ResT>
int Node<DataT, ResT>::getSize() const
{
    return size_;
}


template<typename DataT, Res ResT>
OctantBase* Node<DataT, ResT>::getChild(const int child_idx)
{
    return children_ptr_[child_idx];
}


template<typename DataT, Res ResT>
const OctantBase* Node<DataT, ResT>::getChild(const int child_idx) const
{
    return children_ptr_[child_idx];
}


template<typename DataT, Res ResT>
OctantBase* Node<DataT, ResT>::setChild(const int child_idx, OctantBase* child_ptr)
{
    children_mask_ |= 1 << child_idx;
    std::swap(child_ptr, children_ptr_[child_idx]);
    return child_ptr;
}


template<typename DataT, Res ResT>
Eigen::Vector3i Node<DataT, ResT>::getChildCoord(const int child_idx) const
{
    const Eigen::Vector3i child_offset((child_idx & 1) != 0, (child_idx & 2) != 0, (child_idx & 4) != 0);
    const int child_size = getSize() / 2;
    return getCoord() + child_size * child_offset;
}

} // namespace se

#endif // SE_NODE_IMPL_HPP
