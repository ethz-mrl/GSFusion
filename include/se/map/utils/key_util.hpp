/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2023 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_KEY_UTIL_HPP
#define SE_KEY_UTIL_HPP

#include <algorithm>

#include "se/map/utils/setup_util.hpp"
#include "se/map/utils/type_util.hpp"


namespace se {

#define NUM_DIM 3
#define SCALE_OFFSET 5
#define KEY_SCALE_LIMIT 19

/**
  #define NUM_DIM 3
  uint64_t MASK[64];
  MASK[0] = 0x1c0000000000000;

  for (int i = 1; i < 19; ++i) {
    MASK[i] = MASK[i-1] | (MASK[0] >> (i*3));
  }

  for (int i = 18; i >= 0; --i) {
    std::bitset<64> b(MASK[i]);
    std::cout << "0x" << std::hex << b.to_ullong() << "," << std::endl;
  }
 */
constexpr uint64_t CODE_MASK[] = { ///< Get the code mask for a given scale
    0x1ffffffffffffff,
    0x1fffffffffffff8,
    0x1ffffffffffffc0,
    0x1fffffffffffe00,
    0x1fffffffffff000,
    0x1ffffffffff8000,
    0x1fffffffffc0000,
    0x1ffffffffe00000,
    0x1ffffffff000000,
    0x1fffffff8000000,
    0x1ffffffc0000000,
    0x1fffffe00000000,
    0x1fffff000000000,
    0x1ffff8000000000,
    0x1fffc0000000000,
    0x1ffe00000000000,
    0x1ff000000000000,
    0x1f8000000000000,
    0x1c0000000000000};

constexpr uint64_t SCALE_MASK = 0x1F; ///< 11 111

namespace keyops {

/**
 * \brief Verify if a key is valid.
 *
 * \param[in] key The key to be varified
 * \return True if the key is valid, False otherwise.
 */
inline bool is_valid(const se::key_t key, const se::scale_t limit = KEY_SCALE_LIMIT);

/**
 * \brief Verify if a coordinate can be expressed in a key.
 *
 * \param[in] coord The 3D coordinates to be varified
 * \return True if the coordinates are valid, False otherwise.
 */
inline bool is_valid(const Eigen::Vector3i& coord);

/**
 * \brief Expands a value that can be expressed by <= 19 bits
 *        E.g. DCBA => 00D 00C 00B 00A
 *
 * \param[in] value The value to be expanded
 * \return The expanded value
 */
inline se::code_t expand(unsigned long long value);

/**
 * \brief Compresses a value
 *        E.g. 00D 00C 00B 00A => DCBA
 *
 * \param[in] value The value to be compressed
 * \return The compressed value
 */
inline se::key_t compact(uint64_t value);

/**
 * \brief Encodes given coordinates and scale in a key.
 *
 * \note  The key will only hold as much detail as possible at the given scale.
 *        I.e. if the coordinates are at a higher resolution than the scale can represent,
 *        the details will be lost.
 *
 * \param[in]  coord The coordinates to be encoded
 * \param[in]  scale The scale at which to encode the coordinates
 * \param[out] key   The encoded key
 *
 * \return True if no detail is lost, false otherwise
 */
inline bool encode_key(const Eigen::Vector3i& coord, const se::scale_t scale, se::key_t& key);

/**
 * \brief Encodes given coordinates and scale in a key.
 *
 * \note  The key will only hold as much detail as possible at the given scale.
 *        I.e. if the coordinates are at a higher resolution than the scale can represent,
 *        the details will be lost.
 *
 * \param[in]  coord The coordinates to be encoded
 * \param[in]  scale The scale at which to encode the coordinates
 *
 * \return The encoded key
 */
inline se::key_t encode_key(const Eigen::Vector3i& coord, const se::scale_t scale);

/**
 * \brief Encodes given morton code and scale in a key.
 *
 * \note  The key will only hold as much detail as possible at the given scale.
 *        I.e. if the code is at a higher resolution than the scale can represent,
 *        the details will be lost.
 *
 * \param[in]  coord The coordinates to be encoded
 * \param[in]  scale The scale at which to encode the cooridinates
 * \param[out] key   The encoded key
 *
 * \return True if no detail is lost, false otherwise
 */
inline bool encode_key(const se::key_t code, const se::scale_t scale, se::key_t& key);

/**
 * \brief Encodes given morton code and scale in a key.
 *
 * \note  The key will only hold as much detail as possible at the given scale.
 *        I.e. if the code is at a higher resolution than the scale can represent,
 *        the details will be lost.
 *
 * \param[in]  coord The coordinates to be encoded
 * \param[in]  scale The scale at which to encode the cooridinates
 *
 * \return The encoded key
 */
inline se::key_t encode_key(const se::key_t code, const se::scale_t scale);

/**
 * \brief Extracts the 3D coordinates and scale from a given key.
 *
 * \param[in]  key      The key to be encoded
 * \param[out] coord    The 3D coordinates of the key
 * \param[out] scale    The the scale of the key
 */
inline void decode_key(const se::key_t key, Eigen::Vector3i& coord, scale_t& scale);

/**
 * \brief Compute the Morton code for given x,y,z coordinates.
 *
 * \param[in]  coord    The coordinates to be encoded
 * \param[out] code     The Morten code representing the coordinates
 */
inline void encode_code(const Eigen::Vector3i& coord, se::code_t& code);

/**
 * \brief Compute the Morton code for given x,y,z coordinates.
 *
 * \param[in]  coord    The coordinates to be encoded
 * \param[out] code     The Morten code representing the coordinates
 */
inline se::code_t encode_code(const Eigen::Vector3i& coord);

/**
 * \brief Compute the x,y,z coordinates for a given Morton code.
 *
 * \param[in]  code     The code to be encoded
 * \param[out] coord    The coordinates representing the Morton code
 */
inline void decode_code(const se::code_t code, Eigen::Vector3i& coord);


/**
 * \brief Extracts the child index from a Morton code for a given scale.
 *
 * \param code  The full Morton code
 * \param scale The scale at which to extract the child index
 *
 * \return The child index
 */
inline idx_t code_to_child_idx(const se::code_t code, const scale_t scale);

/**
 * \brief Reduce a key to only its Morton code.
 *
 * \param[in] key The key containing the Morton code
 *
 * \return The Morton code of the key
 */
inline se::code_t key_to_code(const se::key_t key);

/**
 * \brief Reduce a key to only its Morton code.
 *
 * \param[in] key The key containing the Morton code
 *
 * \return The Morton code of the key
 */
inline Eigen::Vector3i key_to_coord(const se::key_t key);

/**
 * \brief Reduce a key to only its scale.
 *
 * \param[in] key The key containing the Morton code
 *
 * \return The scale of the key
 */
inline scale_t key_to_scale(const se::key_t key);

/**
 * \brief For a given key, change the key scale and reduce detail from Morton code up to given the scale.
 *
 * \param[in]  key           The key to be modified
 * \param[in]  scale         The scale the key should be changed to
 * \param[out] key_at_scale  The modified key
 *
 * \return True if the key can be reduced to the given scale, False otherwise
 */
inline bool key_at_scale(const se::key_t key, const se::scale_t scale, se::key_t& key_at_scale);

/**
 * \brief For a given key, reduce detail from Morton code up to given a scale.
 *
 * \param[in]  key           The key containing the code to be modified
 * \param[in]  scale         The scale the code should be reduced to
 * \param[out] code_at_scale The modified code
 *
 * \return True if the code can be reduced to the given scale, False otherwise
 */
inline bool code_at_scale(const se::key_t key, const se::scale_t scale, se::code_t& code_at_scale);

/**
 * \brief Compute the direct parent key for a given key.
 *
 * \param[in] key           The key to compute the direct parent from
 * \param[in] parent_key    The parent key
 */
inline void parent_key(const se::key_t key, se::key_t& parent_key);

/**
 * \brief Removes the voxel position detail within a block from a key while maintainig the scale information
 *
 * \note  The key will not be modified if a node key is handed to it.
 *
 * \param[in]  key              The key to be filtered
 * \param[in]  max_block_scale  The maximum scale of a block
 *
 * \return The filtered block key
 */
inline se::key_t block_key(const se::key_t key, const se::scale_t max_block_scale);

/**
 * \brief Removes the voxel position detail within a block from a code
 *
 * \note  The code will not be modified if a node key is handed to it.
 *
 * \param[in]  key              The key to be filtered
 * \param[in]  max_block_scale  The maximum scale of a block
 *
 * \return The filtered block code
 */
inline se::code_t block_code(const se::key_t key, const se::scale_t max_block_scale);

/**
 * \brief Compute the child key for a given parent key and child index
 *
 * \note `code_at_scale` is equivalent to `se::code_t(child_idx)`
 *
 * \param[in]  parent_key    The key of parent
 * \param[in]  code_at_scale The morton code segment at the scale e.g. 000, 001, ... , 110, ... , 111
 * \param[out] child_key     The key of the child
 */
inline void parent_to_child_key(const se::key_t parent_key, const se::code_t code_at_scale, se::key_t& child_key);

/**
 * \brief Verify if a key is a child of a different key
 *
 * \note  This function is not limited by a single scale change.
 *        If parent_key expresses any ancestor of child_key the function will return True.
 *
 * \param[in] parent_key The key of the parent
 * \param[in] child_key  The key of the potential child
 *
 * \return True if child_key expresses a child node/voxel of parent_key
 */
inline bool is_child(const se::key_t parent_key, const se::key_t child_key);

/**
 * \brief Verify if two keys encode sibling nodes/voxels.
 *
 * \param[in] sibling_1_key The key of the first sibling
 * \param[in] sibling_2_key The key of the second sibling
 *
 * \return True if the keys express siblings nodes/voxels, False otherwise
 */
inline bool is_siblings(const se::key_t sibling_1_key, const se::key_t sibling_2_key);

/**
 * \brief Sorting template. Default small to larger key sorting.
 */
template<Sort = Sort::SmallToLarge>
inline void sort_keys(std::vector<se::key_t>& keys);

/**
 * \brief Sorts the keys from smallest to largest.
 *        - At a given scale a dimension will be prioritised z > y > z (MSB > LSB)
 *        - For equivalent Morton codes smaller scales will be prioritised over larger scales (child first).
 *
 * \param[in/out] keys The keys to be sorted
 */
template<>
inline void sort_keys<Sort::SmallToLarge>(std::vector<se::key_t>& keys);

/**
 * \brief Sorts the keys from largest to smallest.
 *        - At a given scale a dimension will be prioritised z > y > z (MSB > LSB)
 *        - For equivalent Morton codes larger scales will be prioritised over smaller scales (parent first).
 *
 * \param[in/out] keys
 */
template<>
inline void sort_keys<Sort::LargeToSmall>(std::vector<se::key_t>& keys);

/**
 * \brief Filter keys based on the whole key (i.e. code and scale).
 *
 * \param[in]  keys          The keys to be filtered
 * \param[out] unique_keys   The filtered unique keys
 */
template<se::Safe SafeB>
inline void unique_keys(const std::vector<se::key_t>& keys, std::vector<se::key_t>& unique_keys);

/**
 * \brief Filter keys based on their code and keep the key with the smallest scale.
 *
 * \param[in]  keys          The keys to be filtered
 * \param[out] unique_keys   The filtered unique keys
 */
template<se::Safe SafeB>
inline void unique_codes(const std::vector<se::key_t>& keys, std::vector<se::key_t>& unique_keys);

/**
 * \brief Filter keys based on unique allocation.
 *        - Parents will be filtered out
 *        - Voxel position detail within a block will be filtered out
 *
 * \param[in]  keys          The keys to be filtered
 * \param[out] unique_keys   The filtered unique keys
 */
template<se::Safe SafeB>
inline void unique_allocation(const std::vector<se::key_t>& keys, const scale_t max_block_scale, std::vector<se::key_t>& unique_keys);

/**
 * \brief Filter keys at a given scale.
 *        - Keys at finer scales will be moved to the given scale.
 *        - Keys at coarser scales will be removed.
 *
 * \param[in]  keys          The keys to be filtered
 * \param[in]  scale         The scale at which to filter the keys
 * \param[out] unique_keys   The filtered unique keys
 */
template<se::Safe SafeB>
inline void unique_at_scale(const std::vector<se::key_t>& keys, const se::scale_t scale, std::vector<se::key_t>& unique_keys);

/**
 * TODO: 6-connectivity + centre
 * \brief Compute the 6 face neighbour keys of a given key (excluding the key itself)
 *
 * \param[in]  key                 The key of which to get the neighbours from
 * \param[out] face_neighbour_keys The 6 face neighbour keys
 */
inline void face_neighbours(const se::key_t key, std::array<se::key_t, 6> face_neighbour_keys);

/**
 * TODO: 26-connectivity
 * \brief Compute the 26 neighbour keys of a given key (excluding the key itself)
 *
 * \param[in]  key            The key of which to get the neighbours from
 * \param[out] neighbour_keys The 26 neighbour keys
 */
inline void neighbours(const se::key_t key, std::array<se::key_t, 26> neighbour_keys);

/**
 * TODO: 4 siblings (includes key)
 * \brief Get the 7 sibling keys of a given key (including the key itself)
 *
 * \param[in]  key          The key of which to get the siblings from
 * \param[out] sibling_keys The eight sibling keys
 */
inline void siblings(const se::key_t key, std::array<se::key_t, 8> sibling_keys);

} // namespace keyops
} // namespace se

#include "impl/key_util_impl.hpp"

#endif // SE_KEY_UTIL_HPP
