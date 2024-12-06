/*
 * SPDX-FileCopyrightText: 2011-2013 Gerhard Reitmayr, TU Graz
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

/*

 This code is licensed under the MIT License.

 */

#ifndef SE_PERFSTATS_HPP
#define SE_PERFSTATS_HPP

#ifdef __APPLE__
#    include <mach/clock.h>
#    include <mach/mach.h>
#endif


#include <Eigen/Geometry>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <numeric>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

struct PerfStats {
    enum Type { BOOL, COORDINATES, COUNT, CURRENT, DISTANCE, DOUBLE, DURATION, ENERGY, FRAME, FREQUENCY, INT, ITERATION, MEMORY, ORIENTATION, PERCENTAGE, POSITION, POWER, TIME, UNDEFINED, VOLTAGE };

    struct Stats {
        /// Static functions to summarise data of one iteration
        /**
         * \brief Compute the mean value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The mean of the values in the iter_data_vec.
         */
        static double meanIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Provide the last value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The last value in the iter_data_vec.
         */
        static double lastIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Compute the minimum value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The minimum value of the values in the iter_data_vec.
         */
        static double minIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Compute the maximum value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The maximum value of the values in the iter_data_vec.
         */
        static double maxIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Compute the sum of the values of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The sum of the values in the iter_data_vec.
         */
        static double sumIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Merge the stats data of one interation into a scalar value.
         *        The merging strategy depends on the PerfStats::Type.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The merged scalar value of the values in the iter_data_vec.
         */
        static double mergeIter(const std::vector<double>& iter_data_vec, const Type type);


        /// Functions to summarise data of one iteration
        /**
         * \brief Compute the mean value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The mean of the values of the given iteration.
         */
        double meanIter(const size_t iter);

        /**
         * \brief Provide the last value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The last value of the given iteration.
         */
        double lastIter(const size_t iter);

        /**
         * \brief Compute the minimum value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The minimum value of the given iteration.
         */
        double minIter(const size_t iter);

        /**
         * \brief Compute the maximum value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The maximum value of the given iteration.
         */
        double maxIter(const size_t iter);

        /**
         * \brief Compute the sum value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The sum of the values of the given iteration.
         */
        double sumIter(const size_t iter);

        /**
         * \brief Merge the stats data a given interation into a scalar value.
         *        The merging strategy depends on the PerfStats::Type.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The merged scalar value of the values of the given iteration.
         */
        double mergeIter(const size_t iter);


        /// Functions to summarise data of the entire stats
        /**
         * \brief Compute the mean value of all stats data stored so far.
         *
         * \return The mean of all stats data
         */
        double mean() const;

        /**
         * \brief Provide the last stored value to the stats data.
         *
         * \return The last value of the stats data
         */
        double last() const;

        /**
         * \brief Compute the minimum value of all stats data stored so far.
         *
         * \return The minimum value of all stats data.
         */
        double min() const;

        /**
         * \brief Compute the maximum value of all stats data stored so far.
         *
         * \return The maximum value of all stats data.
         */
        double max() const;

        /**
         * \brief Compute the sum of all stats data stored so far.
         *
         * \return The sum of all stats data.
         */
        double sum() const;

        /**
         * \brief Merge all stats data into a scalar value.
         *        The merging strategy depends on the PerfStats::Type.
         *
         * \return The merged scalar value of all stats data.
         */
        double merge() const;


        /**
         * \return The unit of the stats type as a std::string, e.g. "[V]" for PerfStats::Voltage.
         */
        std::string unitString();

        // <iteration/frame, vector of values at iteration/frame>
        std::map<size_t, std::vector<double>> data_;
        bool detailed_;        ///< Flag indicating if the stat should be excluded from basic string output.
        double last_absolute_; ///< The last absolute time the stat data was updated.
        std::mutex mutex_;
        Type type_; ///< The type of data stored in the stat struct.
    };

    struct Results {
        double mean;
        double min;
        double max;
        double sum;
    };

    PerfStats();

    PerfStats(const bool include_detailed);

    /**
     * \brief
     *
     * \return The tab separated stats names with units.
     */
    std::string createHeaderString();

    /**
     * \brief
     *
     * \return
     */
    std::string createDataIterString();

    /**
     * \brief
     *
     * \param[in] iter
     * \return
     */
    std::string createDataIterString(const size_t iter);

    /**
     * \brief
     *
     * \return
     */
    std::string createDataString();

    /**
     * \brief
     *
     * \return
     */
    void debug();

    /**
     * \brief
     *
     * \param[in] key The key to the requested stats.
     * \return The stats cooresponding
     */
    const Stats& get(const std::string& key) const
    {
        return stats_.find(key)->second;
    }

    /**
     * \brief
     *
     * \param[in] key
     * \return
     */
    std::vector<double> getLastData(const std::string& key);

    /**
     * \brief
     *
     * \param[in]
     * \return
     */
    double getLastDataMerged(const std::string& key);

    /**
     * \brief
     *
     * \param[in]
     * \return
     */
    double getSampleTime(const std::string& key);

    /**
     * \brief
     *
     * \return
     */
    static double getTime();

    /**
     * \brief
     *
     * \param[in]
     * \return
     */
    Type getType(const std::string& key);

    /**
     * \brief
     *
     * \return
     */
    void reset();

    /**
     * \brief
     *
     * \param[in]
     * \return
     */
    void reset(const std::string& key);

    /**
     * \brief
     *
     * \param[in] key
     * \param[in] value
     * \param[in] type
     * \param[in] detailed
     * \return
     */
    double sample(const std::string& key, const double value, const Type type = COUNT, const bool detailed = false);

    double sampleT_WB(const Eigen::Matrix4f& T_WB, const bool detailed = false);

    /**
     * \brief
     *
     * \param[in] key
     * \param[in] detailed
     * \return
     */
    double sampleDurationStart(const std::string& key, const bool detailed = false);
    /**
     * \brief
     *
     * \param[in] key
     * \return
     */
    double sampleDurationEnd(const std::string& key);

    /**
     * \brief
     *
     * \param[in] stats_stream
     */
    void setFilestream(std::ofstream* filestream);

    /**
     * \brief Set flag to include detailed stats to std::string output.
     *
     * \param[in] include_detailed
     */
    void includeDetailed(const bool include_detailed)
    {
        if (include_detailed != include_detailed_) {
            filestream_aligned_ = false;
            ostream_aligned_ = false;
        }
        include_detailed_ = include_detailed;
    };

    /**
     * \brief Set the current iteration and add it to the stats.
     *
     * \param[in] iter
     */
    void setIter(const size_t iter)
    {
        iter_ = iter;
        sample("iteration", iter, ITERATION);
    };

    /**
     * \brief Write performance stats to filestream.
     *        The first time the function is called the header will be added.
     *        If the
     */
    void writeToFilestream();

    /**
     * \brief
     *
     * \param[in] ostream
     */
    void writeToOStream(std::ostream& ostream);

    /**
     * \brief
     *
     * \param[in] ostream
     * \param[in] include_iter_data
     */
    void writeSummaryToOStream(std::ostream& ostream, const bool include_iter_data = true);

    std::vector<PerfStats::Type> header_order_ = {FRAME,
                                                  ITERATION,
                                                  TIME,
                                                  DURATION,
                                                  MEMORY,
                                                  POSITION,
                                                  ORIENTATION,
                                                  DISTANCE,
                                                  FREQUENCY,
                                                  BOOL,
                                                  POWER,
                                                  ENERGY,
                                                  CURRENT,
                                                  VOLTAGE,
                                                  COUNT,
                                                  INT,
                                                  DOUBLE,
                                                  PERCENTAGE,
                                                  UNDEFINED}; ///< The order the different types are added to the output
    bool include_detailed_;                                   ///< Flag to add stats marked as detailed to the output

    int insertion_idx_;                  ///< The index of the next stat to be inserted to performance stats
    size_t iter_;                        ///< The current iteration
    std::map<int, std::string> order_;   ///< The order the stats are added to the stats_ map | map idx -> stat name
    std::map<std::string, Stats> stats_; ///< The map stat name -> stat

    /// IO function
    std::ofstream* filestream_;     ///<
    bool filestream_aligned_;       ///<
    size_t filestream_last_iter_;   ///<
    std::streampos filestream_pos_; ///<

    bool ostream_aligned_;     ///<
    size_t ostream_last_iter_; ///<
};

#include "impl/perfstats_impl.hpp"

namespace se {


extern PerfStats perfstats;


} // namespace se

#endif // SE_PERFSTATS_HPP
