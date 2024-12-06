/*
 * SPDX-FileCopyrightText: 2011-2013 Gerhard Reitmayr, TU Graz
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_PERFSTATS_IMPL_HPP
#define SE_PERFSTATS_IMPL_HPP

inline double PerfStats::Stats::meanIter(const std::vector<double>& iter_data_vec)
{
    return sumIter(iter_data_vec) / std::max(iter_data_vec.size(), size_t(1));
}

inline double PerfStats::Stats::lastIter(const std::vector<double>& iter_data_vec)
{
    return iter_data_vec.back();
}

inline double PerfStats::Stats::minIter(const std::vector<double>& iter_data_vec)
{
    return *std::min_element(iter_data_vec.begin(), iter_data_vec.end());
}

inline double PerfStats::Stats::maxIter(const std::vector<double>& iter_data_vec)
{
    return *std::max_element(iter_data_vec.begin(), iter_data_vec.end());
}

inline double PerfStats::Stats::sumIter(const std::vector<double>& iter_data_vec)
{
    return std::accumulate(iter_data_vec.begin(), iter_data_vec.end(), 0.0);
}

inline double PerfStats::Stats::mergeIter(const std::vector<double>& iter_data_vec, const Type type)
{
    switch (type) {
    case COUNT:
        return sumIter(iter_data_vec);
    case CURRENT:
        return maxIter(iter_data_vec);
    case DURATION:
        return sumIter(iter_data_vec);
    case ENERGY:
        return maxIter(iter_data_vec);
    case FREQUENCY:
        return meanIter(iter_data_vec);
    case MEMORY:
        return maxIter(iter_data_vec);
    case POWER:
        return maxIter(iter_data_vec);
    case VOLTAGE:
        return maxIter(iter_data_vec);
    default:
        // { BOOL, DISTANCE, DOUBLE, FRAME, INT, ORIENTATION, POSITION, TIME, UNDEFINED }
        return lastIter(iter_data_vec);
    }
}


inline double PerfStats::Stats::meanIter(const size_t iter)
{
    return sumIter(iter) / std::max(data_[iter].size(), size_t(1));
}

inline double PerfStats::Stats::lastIter(const size_t iter)
{
    return data_[iter].back();
}

inline double PerfStats::Stats::minIter(const size_t iter)
{
    return *std::min_element(data_[iter].begin(), data_[iter].end());
}

inline double PerfStats::Stats::maxIter(const size_t iter)
{
    return *std::max_element(data_[iter].begin(), data_[iter].end());
}

inline double PerfStats::Stats::sumIter(const size_t iter)
{
    return std::accumulate(data_[iter].begin(), data_[iter].end(), 0.0);
}

inline double PerfStats::Stats::mergeIter(const size_t iter)
{
    return mergeIter(data_[iter], type_);
}


inline double PerfStats::Stats::mean() const
{
    double mean = 0;
    for (const auto& iter_data : data_) {
        mean += meanIter(iter_data.second);
    }
    return mean / std::max(data_.size(), size_t(1));
}

inline double PerfStats::Stats::last() const
{
    return (data_.size() > 0) ? lastIter(data_.rbegin()->second) : 0;
}

inline double PerfStats::Stats::min() const
{
    double min = std::numeric_limits<double>::max();
    for (const auto& iter_data : data_) {
        double min_iter = minIter(iter_data.second);
        min = (min_iter < min) ? min_iter : min;
    }
    return min;
}

inline double PerfStats::Stats::max() const
{
    double max = std::numeric_limits<double>::min();
    for (const auto& iter_data : data_) {
        double max_iter = maxIter(iter_data.second);
        max = (max_iter > max) ? max_iter : max;
    }
    return max;
}

inline double PerfStats::Stats::sum() const
{
    double sum = 0;
    for (const auto& iter_data : data_) {
        sum += sumIter(iter_data.second);
    }
    return sum;
}

inline double PerfStats::Stats::merge() const
{
    switch (type_) {
    case COUNT:
        return sum();
    case CURRENT:
        return max();
    case DURATION:
        return sum();
    case ENERGY:
        return max();
    case FREQUENCY:
        return mean();
    case MEMORY:
        return max();
        ;
    case POWER:
        return max();
    case VOLTAGE:
        return max();
    default:
        // { BOOL, DISTANCE, DOUBLE, FRAME, INT, ORIENTATION, POSITION, TIME, UNDEFINED }
        return last();
    }
}


inline std::string PerfStats::Stats::unitString()
{
    switch (type_) {
    case BOOL:
        return "(bool)";
    case DOUBLE:
        return "(double)";
    case COORDINATES:
        return "(voxel)";
    case COUNT:
        return "(count)";
    case CURRENT:
        return "(I)";
    case DISTANCE:
        return "(m)";
    case DURATION:
        return "(s)";
    case ENERGY:
        return "(J)";
    case FRAME:
        return "(#)";
    case FREQUENCY:
        return "(Hz)";
    case INT:
        return "(int)";
    case ITERATION:
        return "(#)";
    case MEMORY:
        return "(MB)";
    case ORIENTATION:
        return "(-)";
    case POSITION:
        return "(m)";
    case POWER:
        return "(W)";
    case TIME:
        return "(s)";
    case VOLTAGE:
        return "(V)";
    default: // { UNDEFINED }
        return "(?)";
    }
}


inline PerfStats::PerfStats() :
        include_detailed_(false), insertion_idx_(0), iter_(SIZE_MAX), filestream_(nullptr), filestream_aligned_(false), filestream_last_iter_(0), ostream_aligned_(false), ostream_last_iter_(0)
{
}


inline PerfStats::PerfStats(const bool include_detailed) :
        include_detailed_(include_detailed),
        insertion_idx_(0),
        iter_(SIZE_MAX),
        filestream_(nullptr),
        filestream_aligned_(false),
        filestream_last_iter_(0),
        ostream_aligned_(false),
        ostream_last_iter_(0)
{
}


inline std::vector<double> PerfStats::getLastData(const std::string& key)
{
    std::map<std::string, Stats>::iterator s = stats_.find(key);
    if (s != stats_.end()) {
        return (s->second.data_.rbegin()->second);
    }

    return std::vector<double>();
}


inline double PerfStats::getLastDataMerged(const std::string& key)
{
    std::map<std::string, Stats>::iterator s = stats_.find(key);
    if (s != stats_.end()) {
        return Stats::mergeIter(s->second.data_.rbegin()->second, s->second.type_);
    }

    return double(0);
}


inline double PerfStats::getSampleTime(const std::string& key)
{
    std::map<std::string, Stats>::iterator s = stats_.find(key);
    if (s != stats_.end()) {
        return s->second.last_absolute_;
    }

    return double(0);
}


inline double PerfStats::getTime()
{
#ifdef __APPLE__
    clock_serv_t cclock;
    mach_timespec_t clockData;
    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
    clock_get_time(cclock, &clockData);
    mach_port_deallocate(mach_task_self(), cclock);
#else
    struct timespec clockData;
    clock_gettime(CLOCK_MONOTONIC, &clockData);
#endif
    return (double) clockData.tv_sec + clockData.tv_nsec / 1000000000.0;
}


inline PerfStats::Type PerfStats::getType(const std::string& key)
{
    std::map<std::string, Stats>::iterator s = stats_.find(key);
    if (s != stats_.end()) {
        return (s->second.type_);
    }

    return (UNDEFINED);
}


inline std::string PerfStats::createHeaderString()
{
    std::stringstream header_ss;

    // Add stats in type order (header_order_)
    for (const auto& type : header_order_) {
        for (const auto& o : order_) {
            std::map<std::string, Stats>::iterator s = stats_.find(o.second);
            if (s == stats_.end() || s->second.detailed_ > include_detailed_) { // if include_detailed == false (0) only include non detailed i.e. detailed == false (0)
                continue;
            }
            if (s->second.type_ == type) {
                header_ss << s->first << " " << s->second.unitString() << "\t"; // s->first := stat name; s->second := stat struct
            }
        }
    }

    std::string header_string = header_ss.str();
    // Erase the trailing TAB.
    header_string.pop_back();
    return header_string;
}


inline std::string PerfStats::createDataIterString()
{
    return createDataIterString(iter_);
}


inline std::string PerfStats::createDataIterString(const size_t iter)
{
    std::stringstream data_ss;
    data_ss << std::fixed << std::setprecision(6);

    for (const auto& type : header_order_) {
        for (const auto& o : order_) {
            std::map<std::string, Stats>::iterator s = stats_.find(o.second);
            if (s == stats_.end() || s->second.detailed_ > include_detailed_) { // if include_detailed == false (0) only include non detailed i.e. detailed == false (0)
                continue;
            }
            if (s->second.type_ == type) {
                std::map<size_t, std::vector<double>>::iterator d = s->second.data_.find(iter);
                if (d != s->second.data_.end()) {
                    data_ss << s->second.mergeIter(s->second.data_[iter], s->second.type_) << "\t";
                }
                else {
                    data_ss << "*\t";
                }
            }
        }
    }

    std::string data_string = data_ss.str();
    // Erase the trailing TAB.
    data_string.pop_back();
    return data_string;
}


inline std::string PerfStats::createDataString()
{
    if (iter_ == SIZE_MAX) {
        return "";
    }
    std::stringstream data_ss;
    for (size_t i = 0; i < iter_; i++) {
        data_ss << createDataIterString(i) << "\n";
    }
    data_ss << createDataIterString(iter_);
    std::string data_string = data_ss.str().c_str();
    return data_string;
}


inline void PerfStats::reset()
{
    stats_.clear();
    filestream_aligned_ = false;
    ostream_aligned_ = false;
}


inline void PerfStats::reset(const std::string& key)
{
    std::map<std::string, Stats>::iterator s = stats_.find(key);
    if (s != stats_.end())
        s->second.data_.clear();
}


inline double PerfStats::sample(const std::string& key, const double value, const Type type, const bool detailed)
{
    double now = getTime();
    Stats& s = stats_[key];

    s.mutex_.lock();
    double last_ = s.last_absolute_;
    if (last_ == 0) {
        order_[insertion_idx_] = key;
        insertion_idx_++;
        filestream_aligned_ = false;
        ostream_aligned_ = false;
    }

    s.data_[iter_].push_back(value);
    s.type_ = type;
    s.last_absolute_ = now;
    s.detailed_ = detailed;

    s.mutex_.unlock();
    return (now);
}


inline double PerfStats::sampleT_WB(const Eigen::Matrix4f& T_WB, const bool detailed)
{
    const Eigen::Vector3f t_WS = T_WB.topRightCorner<3, 1>();
    const Eigen::Quaternionf q_WS(T_WB.topLeftCorner<3, 3>());
    sample("tx", t_WS.x(), POSITION, detailed);
    sample("ty", t_WS.y(), POSITION, detailed);
    sample("tz", t_WS.z(), POSITION, detailed);
    sample("qx", q_WS.x(), ORIENTATION, detailed);
    sample("qy", q_WS.y(), ORIENTATION, detailed);
    sample("qz", q_WS.z(), ORIENTATION, detailed);
    sample("qw", q_WS.w(), ORIENTATION, detailed);
    return getTime();
}


inline double PerfStats::sampleDurationStart(const std::string& key, const bool detailed)
{
    double now = getTime();
    Stats& s = stats_[key];

    s.mutex_.lock();
    double last = s.last_absolute_;
    if (last == 0) {
        order_[insertion_idx_] = key;
        insertion_idx_++;
        s.type_ = DURATION;
        s.detailed_ = detailed;
        filestream_aligned_ = false;
        ostream_aligned_ = false;
    }

    s.last_absolute_ = now;

    s.mutex_.unlock();
    return (now);
}


inline double PerfStats::sampleDurationEnd(const std::string& key)
{
    double now = getTime();
    Stats& s = stats_[key];

    s.mutex_.lock();

    double dur = now - s.last_absolute_;
    s.data_[iter_].push_back(dur);
    s.last_absolute_ = now;

    s.mutex_.unlock();
    return (now);
}


inline void PerfStats::setFilestream(std::ofstream* filestream)
{
    filestream_ = filestream;
    filestream_pos_ = filestream_->tellp();
}


inline void PerfStats::writeToFilestream()
{
    if (filestream_ == nullptr) {
        return;
    }

    if (filestream_aligned_) {
        // Add new data line to table
        for (size_t i = filestream_last_iter_ + 1; i <= iter_; i++) {
            *filestream_ << createDataIterString(i) << std::endl;
        }
    }
    else {
        // Rewrite header whole data table incl. header and data
        filestream_->seekp(filestream_pos_);
        *filestream_ << createHeaderString() << "\n";
        *filestream_ << createDataString() << std::endl;
        filestream_aligned_ = true;
    }
    filestream_last_iter_ = iter_;
}


inline void PerfStats::writeToOStream(std::ostream& ostream)
{
    if (ostream_aligned_) {
        // Write new header
        ostream << createHeaderString() << "\n";
        ostream_aligned_ = true;
    }
    else {
        // Add new data line to table
        for (size_t i = ostream_last_iter_ + 1; i <= iter_; i++) {
            ostream << createDataIterString(i) << std::endl;
        }
    }
    ostream_last_iter_ = iter_;
}


inline void PerfStats::writeSummaryToOStream(std::ostream& ostream, bool include_iter_data)
{
    // Setup ostream parameter
    ostream.precision(10);
    ostream.setf(std::ios::fixed, std::ios::floatfield);

    if (include_iter_data) {
        ostream << createHeaderString() << "\n";
        // Add data lines to table
        for (size_t i = 0; i <= iter_; i++) {
            ostream << createDataIterString(i) << std::endl;
        }
    }

    ostream << "\n\n"; // Create spacing to summary

    struct Results* res = nullptr;
    struct Results* res_ptr = nullptr;
    res_ptr = (struct Results*) malloc(sizeof(struct Results) * stats_.size());
    res = res_ptr; // Set res

    // Set mean, min, max and sum.
    for (const auto& o : order_) {                                               // o := std::map<int, std::string>
        std::map<std::string, Stats>::const_iterator st = stats_.find(o.second); // o.second := stat name string
        if (st == stats_.end()) {                                                // Stat not available
            continue;
        }

        const auto& stat = st->second;
        stat.mean();

        (*res).mean = stat.mean();
        (*res).min = stat.min();
        (*res).max = stat.max();
        (*res).sum = stat.sum();

        ostream << "\"" << st->first << "\" : { ";
        ostream << "\"mean\":\"" << (*res).mean << "\", ";
        ostream << "\"min\":\"" << (*res).min << "\", ";
        ostream << "\"max\":\"" << (*res).max << "\", ";
        ostream << "\"sum\":\"" << (*res).sum << "\"";
        ostream << "}" << std::endl;

        res++; // Increment to next stat res
    }

    free(res_ptr);
    return;
}

#endif // SE_PERFSTATS_IMPL_HPP
