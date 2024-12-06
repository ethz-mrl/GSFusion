/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_CONFIDENCE_IMPL_HPP
#define SE_CONFIDENCE_IMPL_HPP

#include <limits>

namespace se {

template<size_t N, typename T>
template<typename U>
constexpr void Confidence<N, T>::merge(semantics_t _class_id, U weight)
{
    assert((0 <= _class_id && _class_id < this->size()) && "The class ID is valid");
    // Increase the confidence of _class_id and decrease the confidence of all other class IDs.
    for (size_t i = 0; i < _class_id; i++) {
        update_0(i, weight);
    }
    update_1(_class_id, weight);
    for (size_t i = _class_id + 1; i < this->size(); i++) {
        update_0(i, weight);
    }
    class_id = std::distance(this->begin(), std::max_element(this->begin(), this->end()));
}


template<size_t N, typename T>
template<typename U>
constexpr Confidence<N, U> Confidence<N, T>::cast() const
{
    Confidence<N, U> a = {};
    for (size_t i = 0; i < this->size(); i++) {
        a[i] = static_cast<U>((*this)[i]);
    }
    return a;
}


template<size_t N, typename T>
template<typename U,
         std::enable_if_t<(std::is_integral_v<T> && std::is_integral_v<U>) || (std::is_floating_point_v<T> && std::is_floating_point_v<U>), bool>,
         std::enable_if_t<sizeof(T) >= sizeof(U), bool>>
constexpr Confidence<N, T>& Confidence<N, T>::operator+=(const Confidence<N, U>& rhs)
{
    for (size_t i = 0; i < this->size(); i++) {
        (*this)[i] += rhs[i];
    }
    return *this;
}


template<size_t N, typename T>
template<typename U,
         std::enable_if_t<(std::is_integral_v<T> && std::is_integral_v<U>) || (std::is_floating_point_v<T> && std::is_floating_point_v<U>), bool>,
         std::enable_if_t<sizeof(T) >= sizeof(U), bool>>
constexpr Confidence<N, T>& Confidence<N, T>::operator-=(const Confidence<N, U>& rhs)
{
    for (size_t i = 0; i < this->size(); i++) {
        (*this)[i] -= rhs[i];
    }
    return *this;
}


template<size_t N, typename T>
template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool>>
constexpr Confidence<N, T>& Confidence<N, T>::operator*=(U rhs)
{
    for (size_t i = 0; i < this->size(); i++) {
        (*this)[i] *= rhs;
    }
    return *this;
}


template<size_t N, typename T>
template<typename U, std::enable_if_t<std::is_arithmetic_v<U>, bool>>
constexpr Confidence<N, T>& Confidence<N, T>::operator/=(U rhs)
{
    for (size_t i = 0; i < this->size(); i++) {
        (*this)[i] /= rhs;
    }
    return *this;
}


template<size_t N, typename T>
template<typename U>
constexpr void Confidence<N, T>::update_0(semantics_t _class_id, U weight)
{
    (*this)[_class_id] = (*this)[_class_id] * (weight - 1) / weight;
}


template<size_t N, typename T>
template<typename U>
constexpr void Confidence<N, T>::update_1(semantics_t _class_id, U weight)
{
    (*this)[_class_id] = ((*this)[_class_id] * (weight - 1) + UINT8_MAX) / weight;
}

} // namespace se

#endif // SE_CONFIDENCE_IMPL_HPP
