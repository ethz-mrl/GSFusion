// Copyright (c) 2023 Janusch Patas.
// All rights reserved. Derived from 3D Gaussian Splatting for Real-Time Radiance Field Rendering software by Inria and MPII.

#pragma once

#include <torch/torch.h>

#include "gaussian.cuh"
#include "gaussian_utils.cuh"
#include "rasterizer.cuh"

namespace gs {
inline std::tuple<torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>
render(Camera& viewpoint_camera, GaussianModel& gaussianModel, float scaling_modifier = 1.0, torch::Tensor override_color = torch::empty({}))
{
    // Ensure background tensor (bg_color) is on GPU!
    torch::Tensor bg_color = gaussianModel.background.to(torch::kCUDA);

    // Set up rasterization configuration
    GaussianRasterizationSettings raster_settings;
    raster_settings.image_height = static_cast<int>(viewpoint_camera.height);
    raster_settings.image_width = static_cast<int>(viewpoint_camera.width);
    raster_settings.tanfovx = std::tan(viewpoint_camera.fov_x * 0.5f);
    raster_settings.tanfovy = std::tan(viewpoint_camera.fov_y * 0.5f);
    raster_settings.bg = bg_color;
    raster_settings.scale_modifier = scaling_modifier;
    raster_settings.viewmatrix = viewpoint_camera.T_W2C;
    raster_settings.projmatrix = viewpoint_camera.full_proj_matrix;
    raster_settings.sh_degree = gaussianModel.Get_active_sh_degree();
    raster_settings.camera_center = viewpoint_camera.cam_center;
    raster_settings.prefiltered = false;

    GaussianRasterizer rasterizer = GaussianRasterizer(raster_settings);

    auto means3D = gaussianModel.Get_xyz();
    auto means2D = torch::zeros_like(gaussianModel.Get_xyz()).requires_grad_(false);
    auto opacity = gaussianModel.Get_opacity();
    auto scales = gaussianModel.Get_scaling();
    auto rotations = gaussianModel.Get_rotation();
    auto shs = gaussianModel.Get_features();

    auto colors_precomp = torch::Tensor();
    auto cov3D_precomp = torch::Tensor();

    torch::cuda::synchronize();

    // Rasterize visible Gaussians to image, obtain their radii (on screen).
    auto [rendererd_image, radii] = rasterizer.forward(means3D, means2D, opacity, shs, colors_precomp, scales, rotations, cov3D_precomp);

    // Apply visibility filter to remove occluded Gaussians.
    return {rendererd_image, means2D, radii > 0, radii};
}
} // namespace gs
