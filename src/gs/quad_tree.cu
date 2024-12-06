/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <numeric>

#include "gs/quad_tree.cuh"

namespace gs {

cv::Mat Node::getPixels(const cv::Mat& img) const
{
    cv::Rect roi(x0_, y0_, width_, height_);
    return img(roi);
}

float Node::computeError(const cv::Mat& img) const
{
    cv::Mat pixels = getPixels(img);
    cv::Scalar avg_color = cv::mean(pixels);

    std::vector<cv::Mat> channels;
    cv::split(pixels, channels);

    float r_mse = 0.0f, g_mse = 0.0f, b_mse = 0.0f;

    for (int i = 0; i < pixels.rows; i++) {
        for (int j = 0; j < pixels.cols; j++) {
            float r_diff = static_cast<float>(channels[0].at<uchar>(i, j)) - avg_color[0];
            float g_diff = static_cast<float>(channels[1].at<uchar>(i, j)) - avg_color[1];
            float b_diff = static_cast<float>(channels[2].at<uchar>(i, j)) - avg_color[2];

            r_mse += r_diff * r_diff;
            g_mse += g_diff * g_diff;
            b_mse += b_diff * b_diff;
        }
    }

    int count = pixels.rows * pixels.cols;
    r_mse /= count;
    g_mse /= count;
    b_mse /= count;

    float error = r_mse * 0.2989 + g_mse * 0.5870 + b_mse * 0.1140;

    return error * img.rows * img.cols / 90000000.0;
}

void QTree::subdivide()
{
    recursive_subdivide(root_, threshold_, min_pixel_size_, img_);
    all_children_ = find_children(root_);
}


void QTree::renderImg(int thickness, cv::Scalar color)
{
    cv::Mat imgc;
    cv::cvtColor(img_, imgc, cv::COLOR_RGB2BGR);
    cv::imshow("before", imgc);

    std::vector<Node> children = find_children(root_);
    std::cout << "Find " << children.size() << " nodes" << std::endl;

    for (const auto& child : children) {
        cv::Mat pixels = child.getPixels(img_);

        cv::Scalar avg_color = cv::mean(pixels);
        int avg_b = static_cast<int>(std::floor(avg_color[0]));
        int avg_g = static_cast<int>(std::floor(avg_color[1]));
        int avg_r = static_cast<int>(std::floor(avg_color[2]));

        imgc(cv::Rect(child.getOriginX(), child.getOriginY(), child.getWidth(), child.getHeight())).setTo(cv::Scalar(avg_r, avg_g, avg_b));
        if (thickness > 0) {
            cv::rectangle(imgc, cv::Point(child.getOriginX(), child.getOriginY()), cv::Point(child.getOriginX() + child.getWidth(), child.getOriginY() + child.getHeight()), color, thickness);
        }
    }

    cv::imshow("after", imgc);
    cv::waitKey(0);
}

void recursive_subdivide(Node& node, float threshold, int min_pixel_size, cv::Mat& img)
{
    if (node.computeError(img) <= threshold) {
        return;
    }

    int w1 = static_cast<int>(std::floor(node.getWidth() / 2.0));
    int w2 = static_cast<int>(std::ceil(node.getWidth() / 2.0));
    int h1 = static_cast<int>(std::floor(node.getHeight() / 2.0));
    int h2 = static_cast<int>(std::ceil(node.getHeight() / 2.0));

    if (w1 <= min_pixel_size || h1 <= min_pixel_size) {
        return;
    }

    // top left
    Node n1(node.getOriginX(), node.getOriginY(), w1, h1);
    recursive_subdivide(n1, threshold, min_pixel_size, img);
    // bottom left
    Node n2(node.getOriginX(), node.getOriginY() + h1, w1, h2);
    recursive_subdivide(n2, threshold, min_pixel_size, img);
    // top right
    Node n3(node.getOriginX() + w1, node.getOriginY(), w2, h1);
    recursive_subdivide(n3, threshold, min_pixel_size, img);
    // bottom right
    Node n4(node.getOriginX() + w1, node.getOriginY() + h1, w2, h2);
    recursive_subdivide(n4, threshold, min_pixel_size, img);

    std::vector<Node> children{n1, n2, n3, n4};
    node.children = children;
}

std::vector<Node> find_children(const Node& node)
{
    if (node.children.empty()) {
        return {node};
    }
    else {
        std::vector<Node> all_children;
        for (const auto& child : node.children) {
            auto grandchildren = find_children(child);
            all_children.insert(all_children.end(), grandchildren.begin(), grandchildren.end());
        }
        return all_children;
    }
}

} // namespace gs