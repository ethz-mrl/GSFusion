/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_BLOCK_HPP
#define SE_BLOCK_HPP

namespace se {

// Forward declare Node to avoid depending on the order of includes.
template<typename DataT, Res ResT>
class Node;


/**
 * \brief The base used for single-resolution blocks
 */
template<typename DerivedT, typename DataT, int BlockSize>
class BlockSingleRes {
    public:
    typedef DataT DataType;

    BlockSingleRes(const DataType init_data = DataType());

    const DataType& getData(const int voxel_idx) const;

    DataType& getData(const int voxel_idx);

    const DataType& getData(const Eigen::Vector3i& voxel_coord) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord);

    void setData(const unsigned voxel_idx, const DataT& data);

    void setData(const Eigen::Vector3i& voxel_coord, const DataT& data);


    static int getMinScale()
    {
        return min_scale_;
    }

    static int getCurrentScale()
    {
        return curr_scale_;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    std::array<DataType, BlockSize * BlockSize * BlockSize> block_data_;
    static constexpr int min_scale_ = 0;
    static constexpr int curr_scale_ = 0;

    DerivedT& underlying()
    {
        return static_cast<DerivedT&>(*this);
    }
    const DerivedT& underlying() const
    {
        return static_cast<const DerivedT&>(*this);
    }
};


template<typename DataT, int BlockSize, typename DerivedT>
class BlockMultiRes {
};


template<Field FldT, Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
class BlockMultiRes<Data<FldT, ColB, SemB>, BlockSize, DerivedT> {
};


template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
class BlockMultiRes<Data<Field::TSDF, ColB, SemB>, BlockSize, DerivedT> {
    public:
    typedef Data<Field::TSDF, ColB, SemB> DataType;
    typedef DeltaData<Field::TSDF, ColB, SemB> PropDataType;

    BlockMultiRes(const DataType init_data = DataType());

    struct DataUnion {
        DataUnion(){};
        Eigen::Vector3i coord;
        int scale;

        DataType data;
        PropDataType prop_data;

        int data_idx;
        int prop_data_idx;
    };

    /// Get voxel index

    int getVoxelIdx(const Eigen::Vector3i& voxel_coord) const;

    int getVoxelIdx(const Eigen::Vector3i& voxel_coord, const int scale) const;

    /// Get coarsest block data

    const DataType& getData() const;

    DataType& getData();

    /// Get data at current scale

    const DataType& getData(const int voxel_idx) const;

    DataType& getData(const int voxel_idx);

    const DataType& getData(const Eigen::Vector3i& voxel_coord) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord);

    /// Get data at current scale or coarser

    const DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);

    /// Get data at scale

    const DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale);

    /// Get data

    const DataUnion getDataUnion(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataUnion getDataUnion(const Eigen::Vector3i& voxel_coord, const int scale);

    /// Set data at current scale

    void setData(const int voxel_idx, const DataType& data);

    void setData(const Eigen::Vector3i& voxel_coord, const DataType& data);

    void setData(const Eigen::Vector3i& voxel_coord, const int scale, const DataType& data);

    void setDataUnion(const DataUnion& data_union);


    /// Get scales

    int getMinScale() const
    {
        return min_scale_;
    }

    void setMinScale(const int min_scale)
    {
        min_scale_ = min_scale;
    }

    static constexpr int getMaxScale()
    {
        return max_scale_;
    }

    int getCurrentScale() const
    {
        return curr_scale_;
    }

    void setCurrentScale(const int curr_scale)
    {
        curr_scale_ = curr_scale;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    static constexpr int max_scale_ = math::log2_const(BlockSize);

    static constexpr int compute_num_voxels()
    {
        size_t voxel_count = 0;
        unsigned int size_at_scale = BlockSize;
        while (size_at_scale >= 1) {
            voxel_count += size_at_scale * size_at_scale * size_at_scale;

            size_at_scale = size_at_scale >> 1;
        }
        return voxel_count;
    }

    static constexpr std::array<int, math::log2_const(BlockSize) + 1> compute_size_at_scales()
    {
        std::array<int, math::log2_const(BlockSize) + 1> size_at_scales{};

        int size_at_scale = BlockSize;
        int scale = 0;
        while (size_at_scale >= 1) {
            size_at_scales[scale] = size_at_scale;
            size_at_scale = size_at_scale >> 1;
            scale++;
        }
        return size_at_scales;
    }

    static constexpr std::array<int, math::log2_const(BlockSize) + 1> compute_scale_offsets()
    {
        std::array<int, math::log2_const(BlockSize) + 1> scale_offsets{};

        unsigned int size_at_scale = BlockSize;
        scale_offsets[0] = 0;
        int scale = 1;
        while (size_at_scale > 1) {
            scale_offsets[scale] = scale_offsets[scale - 1] + size_at_scale * size_at_scale * size_at_scale;
            size_at_scale = size_at_scale >> 1;
            scale++;
        }
        return scale_offsets;
    }

    static constexpr int num_voxels_ = compute_num_voxels();

    static constexpr std::array<int, math::log2_const(BlockSize) + 1> size_at_scales_ = compute_size_at_scales();

    static constexpr std::array<int, math::log2_const(BlockSize) + 1> scale_offsets_ = compute_scale_offsets();


    std::array<DataType, num_voxels_> block_data_;
    std::array<PropDataType, num_voxels_> block_prop_data_;

    int min_scale_;
    int curr_scale_;

    DerivedT& underlying()
    {
        return static_cast<DerivedT&>(*this);
    }
    const DerivedT& underlying() const
    {
        return static_cast<const DerivedT&>(*this);
    }
};


// Forward decleration
template<typename DataT,
         Res ResT = Res::Single,
         int BlockSize = 8,
         typename PolicyT = std::enable_if_t<math::is_power_of_two(BlockSize)> ///< Verify that the block size is sensible
         >
class Block;


template<Colour ColB, Semantics SemB, int BlockSize, typename DerivedT>
class BlockMultiRes<Data<Field::Occupancy, ColB, SemB>, BlockSize, DerivedT> {
    public:
    typedef Data<Field::Occupancy, ColB, SemB> DataType;
    typedef DeltaData<Field::Occupancy, ColB, SemB> PropDataType;

    BlockMultiRes(const DataType init_data = DataType());

    BlockMultiRes(const Block<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>& block);

    void operator=(const Block<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>& block);

    ~BlockMultiRes();

    /// Get init block data

    const DataType& getInitData() const
    {
        return init_data_;
    };
    DataType& getInitData()
    {
        return init_data_;
    };

    void setInitData(const DataType& init_data)
    {
        init_data_ = init_data;
    };

    /// Get voxel index

    int getVoxelIdx(const Eigen::Vector3i& voxel_coord) const;

    int getVoxelIdx(const Eigen::Vector3i& voxel_coord, const int scale) const;


    /// Get data at current scale

    const DataType& getData(const Eigen::Vector3i& voxel_coord) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord);

    /// Get data at current scale or coarser

    const DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);

    /// Get data at scale

    const DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& getData(const Eigen::Vector3i& voxel_coord, const int scale);

    const DataType& getMaxData(const Eigen::Vector3i& voxel_coord) const;

    DataType& getMaxData(const Eigen::Vector3i& voxel_coord);

    const DataType& getMaxData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out) const;

    DataType& getMaxData(const Eigen::Vector3i& voxel_coord, const int scale_in, int& scale_out);

    const DataType& getMaxData(const Eigen::Vector3i& voxel_coord, const int scale) const;

    DataType& getMaxData(const Eigen::Vector3i& voxel_coord, const int scale);

    const DataType& getMinData() const
    {
        return min_data_;
    };


    /// Set data at current scale
    void setData(const Eigen::Vector3i& voxel_coord, const DataType& data);

    void setData(const Eigen::Vector3i& voxel_coord, const int scale, const DataType& data);

    void setMaxData(const Eigen::Vector3i& voxel_coord, const DataType& data);


    void setMaxData(const Eigen::Vector3i& voxel_coord, const int scale, const DataType& data);

    void setMinData(const DataType& min_data)
    {
        min_data_ = min_data;
    }

    /**
     * \brief Allocate the mip-mapped scales down to 'new_min_scale'.
     */
    void allocateDownTo(const int new_min_scale = 0);

    /**
     * \brief Delete the mip-mapped scales up to 'new_min_scale'.
     */
    void deleteUpTo(const int new_min_scale);

    /**
     * \brief Get the block's data at the coarsest scale.
     *
     * \warning The function does not return a reference to the data.
     *
     * \return The block's data at the coarsest scale
     */
    const DataType& getData() const
    {
        return block_data_[0][0];
    }

    /**
     * \brief Get the block's max data at the coarsest scale.
     *
     * \warning The function does not return a reference to the data.
     *
     * \return The block's max data at the coarsest scale
     */
    const DataType& getMaxData() const
    {
        return block_max_data_[0][0];
    }

    /**
     * \brief Get the block's value at the coarsest scale.
     *
     * \warning The function does not return a reference to the value.
     *
     * \return The block's value at the coarsest scale
     */
    float meanValue()
    {
        return get_field(getData());
    }

    /**
     * \brief Get the block's max value at the coarsest scale.
     *
     * \warning The function does not return a reference to the value.
     *
     * \return The block's max value at the coarsest scale
     */
    float maxValue()
    {
        return get_field(getMaxData());
    }

    const std::vector<DataType*>& blockData() const
    {
        return block_data_;
    }

    std::vector<DataType*>& blockData()
    {
        return block_data_;
    }

    const std::vector<DataType*>& blockMaxData() const
    {
        return block_max_data_;
    }

    std::vector<DataType*>& blockMaxData()
    {
        return block_max_data_;
    }

    /**
     * \brief Get the number of integrations at the current scale.
     */
    size_t currIntegrCount() const
    {
        return curr_integr_count_;
    }

    /**
     * \brief Get the number of observed voxels at the current scale.
     */
    size_t currObservedCount() const
    {
        return curr_observed_count_;
    } ///<

    /**
     * \brief Increment the number of integrations at the current scale by 1.
     */
    void incrCurrIntegrCount()
    {
        curr_integr_count_++;
    }

    /**
     * \brief Increment the number of observed voxels in at the current scale by 1.
     *
     * \param[in] do_increment The optional flag indicating if the counter should be incremented.
     */
    void incrCurrObservedCount(bool do_increment = true);

    /**
     * \brief Reset the current integration and observation count to 0.
     */
    void resetCurrCount();

    /**
     * \brief When a block is initialised from an observed block (i.e. init_data_.observed == true), set the current
     *        observed count to all voxels observed and the integration count to the nodes value. Otherwise reset the current
     *        count.
     */
    void initCurrCout();

    /**
     * \return The integration scale of the buffer.
     */
    int buffer_scale() const
    {
        return buffer_scale_;
    }
    size_t bufferIntegrCount() const
    {
        return buffer_integr_count_;
    }
    size_t bufferObservedCount() const
    {
        return buffer_observed_count_;
    }

    /**
     * \brief Increment the buffer count if incrementation criterion is met.
     *        I.e. the scale normalised number of observations at the buffer scale >= 95% observations at the current scale.
     */
    void incrBufferIntegrCount(const bool do_increment = true);


    /**
     * \brief Increment the number of observed voxels at the buffers scale by 1.
     *
     * \param[in] do_increment The optional flag indicating if the counter should be incremented.
     */
    void incrBufferObservedCount(const bool do_increment = true);
    /**
     * \brief Reset the buffer integration and observation count to 0.
     */
    void resetBufferCount();

    /**
     *  \brief Reset buffer variables to the initial values and free the buffer data if applicable.
     */
    void resetBuffer();

    /**
     * \brief Init buffer variables.
     *
     * \param[in] buffer_scale The scale the buffer should be initialised at.
     */
    void initBuffer(const int buffer_scale);

    /**
     * \brief Check if the scale should be switched from the current scale to the recommended.
     *
     * \return True is data is switched to recommended scale.
     */
    bool switchData();

    /**
     * \brief Get a `const` reference to the voxel data in the buffer at the voxel coordinates.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_coord The voxel coordinates of the data to be accessed.
     *
     * \return `const` reference to the voxel data in the buffer for the provided voxel coordinates.
     */
    const DataType& bufferData(const Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Get a reference to the voxel data in the buffer at the voxel coordinates.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_coord The voxel coordinates of the data to be accessed.
     *
     * \return Reference to the voxel data in the buffer for the provided voxel coordinates.
     */
    DataType& bufferData(const Eigen::Vector3i& voxel_coord);

    /**
     * \brief Get a `const` reference to the voxel data in the buffer at the voxel index.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_idx The voxel index of the data to be accessed.
     *
     * \return `const` reference to the voxel data in the buffer for the provided voxel index.
     */
    const DataType& bufferData(const int voxel_idx) const
    {
        return buffer_data_[voxel_idx];
    }

    /**
     * \brief Get a reference to the voxel data in the buffer at the voxel index.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_idx The voxel index of the data to be accessed.
     *
     * \return Reference to the voxel data in the buffer for the provided voxel index.
     */
    DataType& bufferData(const int voxel_idx)
    {
        return buffer_data_[voxel_idx];
    }

    /**
     * \brief Get a `const` reference to the mean voxel data at the current scale via the voxel index.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_idx The voxel index of the data to be accessed.
     *
     * \return `const` reference to the voxel data in the buffer for the provided voxel index.
     */
    const DataType& currData(const int voxel_idx) const
    {
        return curr_data_[voxel_idx];
    }

    /**
     * \brief Get a reference to the mean voxel data at the current scale via the voxel index.
     *
     * \warning The function does not not check if the voxel_idx exceeds the array size.
     *
     * \param[in] voxel_idx The voxel index of the data to be accessed.
     *
     * \return Reference to the mean voxel data at the current scale for the provided voxel index.
     */
    DataType& currData(const int voxel_idx)
    {
        return curr_data_[voxel_idx];
    }

    /**
     * \brief Get a pointer to the mean block data array at a given scale.
     *
     * \param[in] scale The scale to return the mean block data array from.
     *
     * \return The pointer to the mean block data array at the provided scale.
     *         Returns a nullptr if the scale smaller than the min allocated scale.
     */
    DataType* blockDataAtScale(const int scale);

    /**
     * \brief Get a pointer to the max block data array at a given scale.
     *
     * \param[in] scale The scale to return the max block data array from.
     *
     * \return The pointer to the max block data array at the provided scale.
     *         Returns a nullptr if the scale smaller than the min allocated scale.
     */
    DataType* blockMaxDataAtScale(const int scale);

    /// Get scales

    int getMinScale() const
    {
        return min_scale_;
    }

    void setMinScale(const int min_scale)
    {
        min_scale_ = min_scale;
    }

    static constexpr int getMaxScale()
    {
        return max_scale_;
    }

    int getCurrentScale() const
    {
        return curr_scale_;
    }

    void setCurrentScale(const int curr_scale)
    {
        curr_scale_ = curr_scale;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    static constexpr int max_scale_ = math::log2_const(BlockSize);

    ///<< Initalise array of data with `init_data_`.
    void initialiseData(DataType* voxel_data, const int num_voxels)
    {
        std::fill(voxel_data, voxel_data + num_voxels, init_data_);
    }

    std::vector<DataType*> block_data_;
    std::vector<DataType*> block_max_data_;

    DataType* curr_data_ = nullptr; ///<< Pointer to the data at the current integration scale.
    size_t curr_integr_count_;      ///<< Number of integrations at that current scale.
    size_t curr_observed_count_;    ///<< Number of observed voxels at the current scale

    /**
     * \brief Rather than switching directly to a different integration scale once the integration scale computation
     *        recommends a different scale, data is continued to be integrated at the current scale and additionally into
     *        a buffer at the recommended scale.
     *        Recommended scale == current scale:
     *            The buffer_data_ points to a `nullptr`
     *        Recommended scale < current scale:
     *            The buffer_data_ points to a independently allocated array of voxel data. The data is initialised with
     *            the parent data at the current integration scale. Once the scale changes the data is inserted into the
     *            block_data_ and block_max_data_ vector.
     *        Recommended scale > current scale:
     *            The buffer_data_ points to according scale in the block_data_ vector. The data integration starts from
     *            the mean up-propagated value. Up until the recommened scale > current scale the mean up-propagation starts
     *            from the recommened scale such that the data is not overwritten by the up-propagation from the current scale.
     *            However the max up-propagation continues from the current integration scale. Once the scale changes the
     *            current_data_ and current_scale_ is set to the buffer setup, the finest scale in the block_data_ and
     *            block_max_data_ is deleted and the new finest scales in the buffers adjusted accordingly.
     *
     * \note  The recommended scale can only differ by +/-1 scale from the current scale.
     *        The overhead of integrating at two different scales is insignificant compared to switching immediately as
     *        the double integration only happens in areas where the recommended integration scale changed and stops
     *        as soon as the criteria for switching to the finer or coarser scale.
     */
    int curr_scale_;
    int min_scale_;

    DataType* buffer_data_ = nullptr; ///<< Pointer to the buffer data.
    int buffer_scale_;                ///<< The scale of the buffer.
    size_t buffer_integr_count_;      ///<< Number of integrations at the buffer scale. \note Is only incremented when 95% of the current observations are reached.
    size_t buffer_observed_count_;    ///<< Number of observed voxels in the buffer.

    DataType min_data_;
    DataType init_data_;

    DerivedT& underlying()
    {
        return static_cast<DerivedT&>(*this);
    }
    const DerivedT& underlying() const
    {
        return static_cast<const DerivedT&>(*this);
    }
};


/**
 * \brief The actual block used in the tree.
 *
 * \note Template defaults set in forward decleration further up
 */
template<typename DataT, Res ResT, int BlockSize, typename PolicyT>
class Block : public OctantBase,
              public std::conditional<ResT == Res::Single, BlockSingleRes<Block<DataT, ResT, BlockSize>, DataT, BlockSize>, BlockMultiRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>>::
                  type ///< Conditional CRTP

{
    public:
    typedef DataT DataType;
    static constexpr int size = BlockSize;
    static constexpr int size_sq = BlockSize * BlockSize;
    static constexpr int size_cu = BlockSize * BlockSize * BlockSize;

    /**
     * \brief Initialise block via parent node
     *
     * \param[in] parent        The shared pointer to the parent node
     * \param[in] child_idx      The child id {0,...,7} in relation to the parent
     * \param[in] init_data     The initial data of the block
     */
    Block(Node<DataT, ResT>* parent_ptr, const int child_idx, const DataT init_data);

    static constexpr int getSize()
    {
        return BlockSize;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // namespace se

#include "impl/block_impl.hpp"

#endif // SE_BLOCK_HPP
