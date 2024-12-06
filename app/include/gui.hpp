/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __GUI_HPP
#define __GUI_HPP

#include <open3d/Open3D.h>

#include "gs/gaussian_utils.cuh"

class GUI {
    public:
    GUI(gs::DataQueue& data_queue, std::atomic<bool>& stop_signal, int width, int height) : data_queue_(data_queue), stop_signal_(stop_signal), img_width_(width), img_height_(height)
    {
    }

    void run();

    private:
    void initWidget();
    void updateScene();
    bool onWindowClose();
    void cleanUp();

    gs::DataQueue& data_queue_;
    std::atomic<bool>& stop_signal_;

    int img_width_;
    int img_height_;

    std::shared_ptr<open3d::visualization::gui::Window> window_;
    std::shared_ptr<open3d::visualization::gui::Widget> gs_panel_;
    std::shared_ptr<open3d::visualization::gui::ImageWidget> gs_widget_;
    std::shared_ptr<open3d::visualization::gui::Label> gs_info_;
    std::shared_ptr<open3d::visualization::gui::Widget> panel_;
    std::shared_ptr<open3d::visualization::gui::ImageWidget> rgb_widget_;
    std::shared_ptr<open3d::visualization::gui::ImageWidget> depth_widget_;
    std::shared_ptr<open3d::visualization::gui::Label> img_info_;
};

#endif