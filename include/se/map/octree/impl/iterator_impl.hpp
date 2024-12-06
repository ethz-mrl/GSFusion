/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_ITERATOR_IMPL_HPP
#define SE_ITERATOR_IMPL_HPP

namespace se {


template<typename DerivedT>
BaseIterator<DerivedT>::BaseIterator()
{
    // Reset to an invalid (end) iterator
    clear();
}


template<typename DerivedT>
BaseIterator<DerivedT>::BaseIterator(OctreeType* octree_ptr) : octree_ptr_(octree_ptr)
{
    // Reset to an invalid (end) iterator
    clear();
}


template<typename DerivedT>
BaseIterator<DerivedT>::BaseIterator(const BaseIterator& other) : octant_stack_(other.octant_stack_)
{
}


template<typename DerivedT>
BaseIterator<DerivedT>& BaseIterator<DerivedT>::operator++()
{
    if (octant_stack_.empty()) {
        clear();
        return *this;
    }
    nextData();
    return *this;
}


template<typename DerivedT>
BaseIterator<DerivedT> BaseIterator<DerivedT>::operator++(int)
{
    if (octant_stack_.empty()) {
        clear();
        return *this;
    }
    BaseIterator<DerivedT> previous_state(*this);
    nextData();
    return previous_state;
}


template<typename DerivedT>
bool BaseIterator<DerivedT>::operator==(const BaseIterator& other) const
{
    // The volume_ is obtained from the current place in the octree so checking
    // it would be redundant.
    return (octant_stack_ == other.octant_stack_ && octant_ == other.octant_);
}


template<typename DerivedT>
bool BaseIterator<DerivedT>::operator!=(const BaseIterator& other) const
{
    return !(*this == other);
}


template<typename DerivedT>
se::OctantBase* BaseIterator<DerivedT>::operator*() const
{
    return octant_;
}


template<typename DerivedT>
void BaseIterator<DerivedT>::init()
{
    if (octree_ptr_ != nullptr) {
        NodeType* root = static_cast<NodeType*>((octree_ptr_)->getRoot());
        if (root != nullptr) {
            // Push the root's children on the stack
            for (unsigned int child_idx = 0; child_idx < 8; ++child_idx) {
                se::OctantBase* child_ptr = root->getChild(child_idx);
                if (child_ptr) {
                    octant_stack_.push(child_ptr);
                }
            }

            // Check if root is part of iterator
            if constexpr (DerivedT::has_ignore_condition == true) {
                if (this->underlying().doIgnore(root)) {
                    nextData();
                    return;
                }
            }

            if (this->underlying().isNext(root)) {
                octant_ = root;
                return;
            }

            // Find the next Volume
            nextData();
        }
    }
}


template<typename DerivedT>
void BaseIterator<DerivedT>::nextData()
{
    while (true) {
        if (octant_stack_.empty()) {
            clear();
            return;
        }
        // Get the data from the top of the stacks
        se::OctantBase* octant = octant_stack_.top();
        // Pop the node since we'll be done with it after this call
        octant_stack_.pop();

        if constexpr (DerivedT::has_ignore_condition == true) {
            if (this->underlying().doIgnore(octant)) {
                continue;
            }
        }

        if (octant != nullptr && !octant->isBlock()) {
            // Non-leaf Node, push all children to the stack
            for (int child_idx = 0; child_idx < 8; child_idx++) {
                se::OctantBase* child_ptr = static_cast<NodeType*>(octant)->getChild(child_idx);
                if (child_ptr) {
                    octant_stack_.push(child_ptr);
                }
            }
            // Then continue until a leaf is found
        }

        if (this->underlying().isNext(octant)) {
            octant_ = octant;
            return;
        }
    }
}


template<typename DerivedT>
void BaseIterator<DerivedT>::clear()
{
    octant_stack_ = std::stack<OctantBase*>();
    octant_ = nullptr;
}

} // namespace se

#endif // SE_ITERATOR_IMPL_HPP
