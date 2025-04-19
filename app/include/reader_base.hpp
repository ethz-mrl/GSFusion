/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef __READER_BASE_HPP
#define __READER_BASE_HPP

#include <Eigen/Core>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <string>

#include "se/common/colour_types.hpp"
#include "se/common/str_utils.hpp"
#include "se/image/image.hpp"
#include "se/map/utils/type_util.hpp"


namespace se {

enum class ReaderType {
    /** Use the se::ReplicaReader. */
    REPLICA,
    /** Use the se::ScanNetppReader. */
    SCANNETPP,
    /** Use the se::TUMReader. */
    TUM,
    UNKNOWN
};

ReaderType string_to_reader_type(const std::string& s);

std::string reader_type_to_string(ReaderType t);


struct ReaderConfig {
    /** The type of the dataset reader to use.
     */
    ReaderType reader_type = se::ReaderType::REPLICA;

    /** The path to the dataset. This might be a path to a file or a directory depending on the
     * reader type.
     */
    std::string sequence_path;

    /** The path to the ground truth file.
     */
    std::string ground_truth_file;

    /** The scaling factor to convert depth values to metres. A value of 0 will use the default
     * scaling for the particular dataset. This only needs to be set when using a modified dataset,
     * e.g. when using a dataset in the TUM format with depth scaled by 1000 instead of the default
     * 5000 for TUM, inverse_scale would need to be set to 1/1000.
     */
    float inverse_scale = 0.0f;

    /** The rate in Hz at which dataset frames are read. A value of 0 will result in reading frames
     * as quickly as possible. If frames can be processed faster than they are read, the program
     * will wait before reading the next frame until the target rate is reached. If frames are
     * processed slower than they are read the behaviour depends on the value of
     * se::ReaderConfig::drop_frames.
     */
    float fps = 0.0f;

    /** Whether to drop frames when they can't be processed fast enough. This option only has an
     * effect if se::ReaderConfig::fps is greater than 0 and frames are read faster than they can be
     * processed:
     * - When false, all frames in the dataset will be processed. This will result in a processing
     * rate lower than what was specified in se::ReaderConfig::fps. This simulates a camera with a
     * framerate of se::ReaderConfig::fps and processing every frame produced by the camera.
     * - When true, input frames will be skipped, assumming they arrive at a rate of
     * se::ReaderConfig::fps. This simulates a camera with a framerate of se::ReaderConfig::fps and
     * always processing the laster frame produced by the camera.
     */
    bool drop_frames = false;

    /** The verbosity level of dataset readers. A positive value results in more information being
     * printed to standard output when reading data. Greater values result in more information being
     * printed.
     */
    int verbose = 0;

    /** Reads the struct members from the "reader" node of a YAML file. Members not present in the
     * YAML file aren't modified.
     */
    void readYaml(const std::string& filename);
};

std::ostream& operator<<(std::ostream& os, const ReaderConfig& c);


/** The result of trying to read a depth/colour image or a pose.
 */
enum class ReaderStatus : int {
    /** Data read successfully. */
    ok = 0,
    /** Temporary data read error.
     * Further reads might succeed. Typically used to indicate an invalid
     * image or pose. */
    skip,
    /** End of dataset reached. */
    eof,
    /** Fatal data read error.
     * No further read should be attempted. Typically used to indicate that
     * the dataset could not be read at all or no camera was found. */
    error,
};

std::ostream& operator<<(std::ostream& os, const ReaderStatus& s);


/** Base abstract class for dataset readers.
 *
 * The overloaded nextData() functions all increment the frame counter, so
 * only one of them should be called within a single pipeline iteration.
 *
 * \note Derived classes should adhere to the following rules:
 *     - The se::Reader constructor should be called at the very beginning of
 *       all derived class constructors.
 *     - se::Reader::restart() should be called at the very beginning of all
 *       derived class implementations of restart().
 *     - The value of se::Reader::is_live_reader_ should be set appropriately
 *       at the derived class constructor.
 *     - The value of se::Reader::status_ should only be set in derived class
 *       constructors and se::Reader::restart() if applicable.
 */
class Reader {
    public:
    /** Construct a Reader from a ReaderConfig.
     *
     * \note This constructor should be called in all derived class
     *       constructors.
     *
     * \param[in] c The configuration struct to use.
     */
    Reader(const ReaderConfig& c);

    virtual ~Reader(){};

    /** Read the next depth image.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image The next depth image.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image);

    /** Read the next depth image and ground truth pose.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image The next depth image.
     * \param[out] T_WB         The next ground truth pose.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Eigen::Matrix4f& T_WB);

    /** Read the next depth and colour images.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image  The next depth image.
     * \param[out] colour_image The next colour image.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Image<rgb_t>& colour_image);

    /** Read the next depth and colour images and ground truth pose.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image  The next depth image.
     * \param[out] colour_image The next colour image.
     * \param[out] T_WB         The next ground truth pose.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Image<rgb_t>& colour_image, Eigen::Matrix4f& T_WB);

    /** Read the next depth and semantic images.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image    The next depth image.
     * \param[out] class_id_image The next colour image.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Image<semantics_t>& class_id_image);

    /** Read the next depth and semantic images and ground truth pose.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image    The next depth image.
     * \param[out] class_id_image The next colour image.
     * \param[out] T_WB           The next ground truth pose.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Image<semantics_t>& class_id_image, Eigen::Matrix4f& T_WB);

    /** Read the next depth, colour and semantic images.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image    The next depth image.
     * \param[out] colour_image   The next colour image.
     * \param[out] class_id_image The next colour image.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Image<rgb_t>& colour_image, Image<semantics_t>& class_id_image);

    /** Read the next depth, colour and semantic images and ground truth pose.
     *
     * \note The frame number is incremented when calling this function.
     *
     * \param[out] depth_image    The next depth image.
     * \param[out] colour_image   The next colour image.
     * \param[out] class_id_image The next colour image.
     * \param[out] T_WB           The next ground truth pose.
     * \return An appropriate status code.
     */
    ReaderStatus nextData(Image<float>& depth_image, Image<rgb_t>& colour_image, Image<semantics_t>& class_id_image, Eigen::Matrix4f& T_WB);

    /** Read the ground truth pose at the provided frame number.
     * Each line in the ground truth file should correspond to a single
     * depth/colour image pair and have a format<br>
     * `... tx ty tz qx qy qz qw`,<br>
     * that is the pose is encoded in the last 7 columns of the line.
     *
     * \param[in]  frame The frame number of the requested ground truth pose.
     * \param[out] T_WB  The ground truth pose.
     * \return An appropriate status code.
     */
    ReaderStatus getPose(Eigen::Matrix4f& T_WB, const size_t frame);

    /** Restart reading from the beginning.
     *
     * \note Although this is a virtual function, it does have a default
     *       implementation that resets the Reader protected variables.
     *       Reader::restart() (this default implementation) should be called
     *       at the very beginning of any implementation of restart().
     */
    virtual void restart() = 0;

    /** The name of the reader.
     *
     * \return The name of the reader as a string.
     */
    virtual std::string name() const = 0;

    /** The state of the reader.
     * If good() returns false, there is no need to keep reading frames.
     *
     * \return True if the last call to one of the Reader::next*() functions
     *         completed successfully.
     */
    bool good() const;

    /** The current frame number.
     * Frame numbering starts from 0, so the number of frames read so far is
     * `frame() + 1`.
     *
     * \return The number of frames read. Returns SIZE_MAX if no frames
     *         have been read yet.
     */
    size_t frame() const;

    /** The total number of frames in the current dataset.
     *
     * \return The total number of frames. Returns 0 if the number of frames
     *         is unknown (e.g. for camera input).
     */
    size_t numFrames() const;

    /** The dimensions of the depth images.
     *
     * \return A 2D vector containing the width and height of the images.
     */
    Eigen::Vector2i depthImageRes() const;

    /** The dimensions of the colour images.
     *
     * \return A 2D vector containing the width and height of the images.
     */
    Eigen::Vector2i colourImageRes() const;

    /** The dimensions of the semantic class ID images.
     *
     * \return A 2D vector containing the width and height of the images.
     */
    Eigen::Vector2i semanticImageRes() const;

    /** Whether the reader uses a live camera as input.
     *
     * \return True if the input is from a live camera, false otherwise.
     */
    bool isLiveReader() const;

    /** Whether the loaded dataset contains colour images.
     *
     * \return True if the loaded dataset contains colour images, false otherwise.
     */
    bool hasColour() const;

    /** Whether the loaded dataset contains semantic class ID images.
     *
     * \return True if the loaded dataset contains semantic class ID images, false otherwise.
     */
    bool hasSemantics() const;

    /** Merge se::ReaderStatus values keeping the worst one.
     *
     * \param[in] status_1 The first se::ReaderStatus to merge.
     * \param[in] status_2 The second se::ReaderStatus to merge.
     * \return The worst of the 2 statuses.
     */
    static ReaderStatus mergeStatus(ReaderStatus status_1, ReaderStatus status_2);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
    std::string sequence_path_;
    std::string ground_truth_file_;
    std::ifstream ground_truth_fs_;
    Eigen::Vector2i depth_image_res_;
    Eigen::Vector2i colour_image_res_;
    Eigen::Vector2i class_id_image_res_;
    float fps_;
    double spf_;
    bool drop_frames_;
    int verbose_;
    bool is_live_reader_;
    ReaderStatus status_;
    /** The frame_ is initialized to SIZE_MAX, so that when first incremented
     * it becomes 0. Unsigned integer overflow is defined behaviour in C/C++
     * so this is safe to do.
     */
    size_t frame_;
    size_t num_frames_;
    bool has_colour_;
    bool has_semantics_;


    /** Read the ground truth pose at the provided frame number.
     * Each line in the ground truth file should correspond to a single
     * depth/colour image pair and have a format<br>
     * `... tx ty tz qx qy qz qw`,<br>
     * that is the pose is encoded in the last 7 columns of the line.
     *
     * \note Use getPose(...) to request a pose. It keeps track of the ground_truth_frame_
     *       and ifstream state.
     *
     * \param[out] T_WB      The ground truth pose.
     * \param[in]  frame     The frame number of the requested ground truth pose.
     * \param[in]  delimiter The character delimiting columns in the file. Defaults to space.
     * \return An appropriate status code.
     */
    ReaderStatus readPose(Eigen::Matrix4f& T_WB, const size_t frame, const char delimiter = ' ');


    /** Read the next ground truth pose.
     * Each line in the ground truth file should correspond to a single
     * depth/colour image pair and have a format<br>
     * `... tx ty tz qx qy qz qw`,<br>
     * that is the pose is encoded in the last 7 columns of the line.
     *
     * \note The frame number is NOT incremented inside this function.
     *
     * \param[out] T_WB The next ground truth pose.
     * \return An appropriate status code.
     */
    virtual ReaderStatus nextPose(Eigen::Matrix4f& T_WB);

    /** Read the next colour image.
     *
     * \param[out] colour_image The next colour image.
     * \return An appropriate status code.
     */
    virtual ReaderStatus nextColour(Image<rgb_t>& colour_image);

    /** Read the next semantics image.
     *
     * \param[out] class_id_image The next per-pixel class ID image.
     * \return An appropriate status code.
     */
    virtual ReaderStatus nextSemantics(Image<semantics_t>& class_id_image);


    private:
    size_t ground_truth_frame_;
    char ground_truth_delimiter_;
    std::chrono::steady_clock::time_point prev_frame_timestamp_;

    /** Prepare for reading the next frame.
     * If Reader::fps_ is 0, this function just increments the frame number
     * by 1. Otherwise it sleeps for the required time so that the desired
     * framerate is achieved. It then increments the frame number by 1 or
     * more, so that the value of Reader::drop_frames_ is respected.
     */
    void nextFrame();

    /** Read the next depth image.
     *
     * \param[out] depth_image The next depth image.
     * \return An appropriate status code.
     */
    virtual ReaderStatus nextDepth(Image<float>& depth_image) = 0;

    ReaderStatus nextData(Image<float>& depth_image, Image<rgb_t>* colour_image, Image<semantics_t>* class_id_image, Eigen::Matrix4f* T_WB);
};

} // namespace se

/** The overview of the reader configuration in configuration format.
 *
 * \return The configuration of the reader to the ostream.
 */
static std::ostream& operator<<(std::ostream& out, se::Reader* reader)
{
    out << se::str_utils::header_to_pretty_str("READER") << "\n";
    out << se::str_utils::str_to_pretty_str(reader->name(), "Reader type") << "\n";
    out << se::str_utils::str_to_pretty_str(((reader->numFrames() == 0) ? "Unknown" : std::to_string(reader->numFrames())), "Number frames") << "\n";
    out << "\n";
    return out;
}


#endif
