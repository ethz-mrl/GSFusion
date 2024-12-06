/*
 * SPDX-FileCopyrightText: 2023 Janusch Patas
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * All rights reserved. Some of the code is derived from 3D Gaussian Splatting for Real-Time Radiance Field Rendering software by Inria and MPII.
 */

#ifndef GS_GAUSSIAN_HPP
#define GS_GAUSSIAN_HPP

#include <filesystem>
#include <torch/torch.h>

#include "gaussian_utils.cuh"

namespace gs {

namespace param {
struct OptimizationParameters {
    int sh_degree = 3;
    float position_lr = 0.00016f;
    float feature_lr = 0.0025f;
    float opacity_lr = 0.05f;
    float scaling_lr = 0.001f;
    float rotation_lr = 0.001f;
    float lambda_dssim = 0.2f;
    float qtree_thresh = 0.1f;
    int qtree_min_pixel_size = 1;
    int kf_thresh = 50;
    int kf_iters = 10;
    int non_kf_iters = 5;
    int random_kf_num = 5;
    int global_iters = 10;
    bool keep_all_frames = false;
};

OptimizationParameters read_optim_params_from_json(const std::string& path);
} // namespace param


class GaussianModel {
    public:
    explicit GaussianModel(const param::OptimizationParameters& params, const std::string& ply_path) : _sh_degree(params.sh_degree), optimParams(params), output_path(ply_path)
    {
    }
    // Copy constructor
    GaussianModel(const GaussianModel& other) = delete;
    // Copy assignment operator
    GaussianModel& operator=(const GaussianModel& other) = delete;
    // Move constructor
    GaussianModel(GaussianModel&& other) = default;
    // Move assignment operator
    GaussianModel& operator=(GaussianModel&& other) = default;

    // Getters
    inline torch::Tensor Get_xyz() const
    {
        return _xyz;
    }
    inline torch::Tensor Get_opacity() const
    {
        return torch::sigmoid(_opacity);
    }
    inline torch::Tensor Get_rotation() const
    {
        return torch::nn::functional::normalize(_rotation);
    }
    inline torch::Tensor Get_scaling()
    {
        return torch::exp(_scaling);
    }
    inline int Get_active_sh_degree() const
    {
        return _sh_degree;
    }
    inline int Get_size() const
    {
        return _xyz.size(0);
    }
    torch::Tensor Get_features() const;
    torch::Tensor Get_covariance(float scaling_modifier = 1.0);

    // Methods
    void Add_gaussians(std::vector<Point>& positions, std::vector<Color>& colors, std::vector<float>& scales);
    void Save_ply(const std::filesystem::path& file_path, int iteration, bool isLastIteration);

    std::unique_ptr<torch::optim::Adam> optimizer;
    param::OptimizationParameters optimParams;
    std::filesystem::path output_path;
    torch::Tensor background = torch::tensor({0.f, 0.f, 0.f});

    private:
    std::vector<std::string> construct_list_of_attributes();
    void training_setup();

    int _sh_degree = 0;
    bool _init_status = false;
    torch::Tensor _xyz;
    torch::Tensor _scaling;
    torch::Tensor _rotation;
    torch::Tensor _opacity;
    torch::Tensor _features_dc;
    torch::Tensor _features_rest;
};

void cat_tensors_to_optimizer(torch::optim::Adam* optimizer, torch::Tensor& extension_tensor, torch::Tensor& old_tensor, int param_position);
} // namespace gs

#endif // GS_GAUSSIAN_HPP