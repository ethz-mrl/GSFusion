/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "gui.hpp"

#include <Eigen/Core>
#include <open3d/visualization/rendering/ColorGrading.h>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <thread>


void GUI::run()
{
    auto& app = open3d::visualization::gui::Application::GetInstance();
    app.Initialize("third_party/open3d/share/resources");
    initWidget();
    app.AddWindow(window_);
    std::thread update_thread([this]() { this->updateScene(); });
    update_thread.detach();
    app.Run();
}


void GUI::initWidget()
{
    window_ = std::make_shared<open3d::visualization::gui::Window>("GSFusion | Online RGB-D Mapping", 1280, 720);
    float em = window_->GetTheme().font_size;

    gs_panel_ = std::make_shared<open3d::visualization::gui::Vert>(0, open3d::visualization::gui::Margins(0.5f * em));
    gs_panel_->AddChild(std::make_shared<open3d::visualization::gui::Label>("Rendered RGB"));
    gs_info_ = std::make_shared<open3d::visualization::gui::Label>("Online optimization");
    gs_panel_->AddChild(gs_info_);

    auto black_img0 = std::make_shared<open3d::geometry::Image>();
    black_img0->Prepare(img_width_, img_height_, 3, 1);
    std::fill(black_img0->data_.begin(), black_img0->data_.end(), 0);
    gs_widget_ = std::make_shared<open3d::visualization::gui::ImageWidget>(black_img0);
    gs_panel_->AddChild(gs_widget_);

    window_->AddChild(gs_panel_);

    panel_ = std::make_shared<open3d::visualization::gui::Vert>(0, open3d::visualization::gui::Margins(0.5f * em));
    panel_->AddChild(std::make_shared<open3d::visualization::gui::Label>("Input RGB-D sequence"));
    img_info_ = std::make_shared<open3d::visualization::gui::Label>("Image ID: ---");
    panel_->AddChild(img_info_);

    // Add RGB and depth widgets
    auto black_img1 = std::make_shared<open3d::geometry::Image>();
    black_img1->Prepare(img_width_, img_height_, 3, 1);
    std::fill(black_img1->data_.begin(), black_img1->data_.end(), 1);
    rgb_widget_ = std::make_shared<open3d::visualization::gui::ImageWidget>(black_img1);
    panel_->AddChild(rgb_widget_);

    auto black_img2 = std::make_shared<open3d::geometry::Image>();
    black_img2->Prepare(img_width_, img_height_, 3, 1);
    std::fill(black_img2->data_.begin(), black_img2->data_.end(), 2);
    depth_widget_ = std::make_shared<open3d::visualization::gui::ImageWidget>(black_img2);
    panel_->AddChild(depth_widget_);

    window_->AddChild(panel_);

    // Set layout
    float gs_width_ratio_ = 0.66;
    auto contentRect = window_->GetContentRect();
    int gs_width = static_cast<int>(contentRect.width * gs_width_ratio_);
    panel_->SetFrame(open3d::visualization::gui::Rect(contentRect.x, contentRect.y, contentRect.width - gs_width, contentRect.height));
    gs_panel_->SetFrame(open3d::visualization::gui::Rect(panel_->GetFrame().GetRight(), contentRect.y, gs_width, contentRect.height));
    gs_info_->SetFrame(open3d::visualization::gui::Rect(panel_->GetFrame().GetRight(), contentRect.y, gs_width, em));

    window_->SetOnClose([this]() { return this->onWindowClose(); });
}


void GUI::updateScene()
{
    while (!stop_signal_.load()) {
        if (data_queue_.getSize() == 0) {
            continue;
        }
        gs::DataPacket data_packet = data_queue_.pop();

        auto vis_rgb = std::make_shared<open3d::geometry::Image>();
        auto vis_depth = std::make_shared<open3d::geometry::Image>();
        auto vis_rendered_rgb = std::make_shared<open3d::geometry::Image>();
        vis_rgb->Prepare(data_packet.rgb.cols, data_packet.rgb.rows, 3, 1);
        vis_depth->Prepare(data_packet.depth.cols, data_packet.depth.rows, 3, 1);
        vis_rendered_rgb->Prepare(data_packet.rendered_rgb.cols, data_packet.rendered_rgb.rows, 3, 1);
        memcpy(vis_rgb->data_.data(), data_packet.rgb.data, vis_rgb->data_.size());
        memcpy(vis_depth->data_.data(), data_packet.depth.data, vis_depth->data_.size());
        memcpy(vis_rendered_rgb->data_.data(), data_packet.rendered_rgb.data, vis_rendered_rgb->data_.size());

        std::ostringstream gs_text;
        std::ostringstream img_text;
        if (data_packet.ID > 0) {
            gs_text << std::fixed << std::setprecision(2) << "Online optimization | "
                    << "#Gaussians=" << data_packet.num_splats << " | #KF=" << data_packet.num_kf << " | FPS=" << data_packet.fps;
            img_text << "Image ID: " << data_packet.ID;
        }
        else if (data_packet.global_iter > 0) {
            gs_text << "Global optimization | "
                    << "#KF=" << data_packet.num_kf << " | Iter=" << data_packet.global_iter;
            img_text << "Image ID: ---";
        }
        else {
            gs_text << "Start global optimization...";
            img_text << "Image ID: ---";
        }

        gs_info_->SetText(gs_text.str().c_str());
        img_info_->SetText(img_text.str().c_str());

        open3d::visualization::gui::Application::GetInstance().PostToMainThread(window_.get(), [this, &vis_rgb, &vis_depth, &vis_rendered_rgb]() {
            this->rgb_widget_->UpdateImage(vis_rgb);
            this->depth_widget_->UpdateImage(vis_depth);
            this->gs_widget_->UpdateImage(vis_rendered_rgb);
        });
    }

    cleanUp();
    std::cout << "Both online and offline mapping are finished. You can close the GUI now!" << std::endl;
}


bool GUI::onWindowClose()
{
    if (stop_signal_) {
        return true;
    }
    else {
        return false;
    }
}


void GUI::cleanUp()
{
    window_.reset();
    gs_panel_.reset();
    gs_widget_.reset();
    gs_info_.reset();
    panel_.reset();
    rgb_widget_.reset();
    depth_widget_.reset();
    img_info_.reset();
}