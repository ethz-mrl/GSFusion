/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef GS_QUAD_TREE_HPP
#define GS_QUAD_TREE_HPP

#include <opencv2/opencv.hpp>

namespace gs {
class Node {
    public:
    Node(int x0, int y0, int width, int height) : x0_(x0), y0_(y0), width_(width), height_(height)
    {
    }

    std::vector<Node> children;

    inline int getOriginX() const
    {
        return x0_;
    }

    inline int getOriginY() const
    {
        return y0_;
    }

    inline int getWidth() const
    {
        return width_;
    }

    inline int getHeight() const
    {
        return height_;
    }

    cv::Mat getPixels(const cv::Mat& img) const;
    float computeError(const cv::Mat& img) const;

    private:
    int x0_;
    int y0_;
    int width_;
    int height_;
};

class QTree {
    public:
    QTree(float threshold, int min_pixel_size, cv::Mat& img) : threshold_(threshold), min_pixel_size_(min_pixel_size), img_(img), root_(0, 0, img.cols, img.rows)
    {
    }

    inline std::vector<Node> getAllNodes() const
    {
        return all_children_;
    }

    inline Node getNode(const int n) const
    {
        return all_children_[n];
    }

    void subdivide();
    void renderImg(int thickness = 1, cv::Scalar color = cv::Scalar(0, 0, 0));

    private:
    float threshold_;
    int min_pixel_size_;
    cv::Mat& img_;
    Node root_;
    std::vector<Node> all_children_;
};

void recursive_subdivide(Node& node, float threshold, int min_pixel_size, cv::Mat& img);

std::vector<Node> find_children(const Node& node);

} // namespace gs

#endif // GS_QUAD_TREE_HPP