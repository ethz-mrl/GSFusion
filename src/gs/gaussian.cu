/*
 * SPDX-FileCopyrightText: 2023 Janusch Patas
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Jiaxin Wei
 * All rights reserved. Some of the code is derived from 3D Gaussian Splatting for Real-Time Radiance Field Rendering software by Inria and MPII.
 */

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

#include "gs/gaussian.cuh"
#include "gs/gaussian_utils.cuh"

using Slice = torch::indexing::Slice;

namespace gs {

namespace param {
OptimizationParameters read_optim_params_from_json(const std::string& path)
{
    std::filesystem::path json_path = path;
    // Check if the file exists before trying to open it
    if (!std::filesystem::exists(json_path)) {
        throw std::runtime_error("Error: " + json_path.string() + " does not exist!");
    }

    std::ifstream file(json_path);
    if (!file.is_open()) {
        throw std::runtime_error("OptimizationParameter file could not be opened.");
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string jsonString = buffer.str();
    file.close();

    // Parse the JSON string
    nlohmann::json json = nlohmann::json::parse(jsonString);

    OptimizationParameters params;
    params.sh_degree = json["sh_degree"];
    params.position_lr = json["position_lr"];
    params.feature_lr = json["feature_lr"];
    params.opacity_lr = json["opacity_lr"];
    params.scaling_lr = json["scaling_lr"];
    params.rotation_lr = json["rotation_lr"];
    params.lambda_dssim = json["lambda_dssim"];
    params.qtree_thresh = json["qtree_thresh"];
    params.qtree_min_pixel_size = json["qtree_min_pixel_size"];
    params.kf_thresh = json["kf_thresh"];
    params.kf_iters = json["kf_iters"];
    params.non_kf_iters = json["non_kf_iters"];
    params.random_kf_num = json["random_kf_num"];
    params.global_iters = json["global_iters"];
    params.keep_all_frames = json["keep_all_frames"];

    return params;
}
} // namespace param


torch::Tensor GaussianModel::Get_features() const
{
    auto features_dc = _features_dc;
    auto features_rest = _features_rest;
    return torch::cat({features_dc, features_rest}, 1);
}


torch::Tensor GaussianModel::Get_covariance(float scaling_modifier)
{
    auto L = build_scaling_rotation(scaling_modifier * Get_scaling(), Get_rotation());
    auto actual_covariance = torch::mm(L, L.transpose(1, 2));
    auto symm = strip_symmetric(actual_covariance);
    return symm;
}


/**
* \param[in]  positions Vector of input positions of gs::Point type
* \param[in]  colors    Vector of input colors of gs::Color type
* \param[in]  scales    Vector of input scales of float type
*/
void GaussianModel::Add_gaussians(std::vector<Point>& positions, std::vector<Color>& colors, std::vector<float>& scales)
{
    const auto pointType = torch::TensorOptions().dtype(torch::kFloat32);
    const auto colorType = torch::TensorOptions().dtype(torch::kUInt8);
    if (_init_status == false) {
        _xyz = torch::from_blob(positions.data(), {static_cast<long>(positions.size()), 3}, pointType).to(torch::kCUDA).set_requires_grad(true);
        _scaling = torch::log(torch::from_blob(scales.data(), {static_cast<long>(positions.size()), 1}, pointType)).repeat({1, 3}).to(torch::kCUDA, true).set_requires_grad(true);
        _rotation = torch::zeros({_xyz.size(0), 4}).index_put_({Slice(), 0}, 1).to(torch::kCUDA, true).set_requires_grad(true);
        _opacity = inverse_sigmoid(0.5 * torch::ones({_xyz.size(0), 1})).to(torch::kCUDA, true).set_requires_grad(true);

        // colors
        auto fused_color = RGB2SH(torch::from_blob(colors.data(), {static_cast<long>(colors.size()), 3}, colorType).to(pointType) / 255.f).to(torch::kCUDA);

        // features
        auto features = torch::zeros({fused_color.size(0), 3, static_cast<long>(std::pow((_sh_degree + 1), 2))}).to(torch::kCUDA);
        features.index_put_({Slice(), Slice(torch::indexing::None, 3), 0}, fused_color);
        _features_dc = features.index({Slice(), Slice(), Slice(0, 1)}).transpose(1, 2).contiguous().set_requires_grad(true);
        _features_rest = features.index({Slice(), Slice(), Slice(1, torch::indexing::None)}).transpose(1, 2).contiguous().set_requires_grad(true);

        training_setup();
        _init_status = true;
    }
    else {
        auto new_xyz = torch::from_blob(positions.data(), {static_cast<long>(positions.size()), 3}, pointType).to(torch::kCUDA).set_requires_grad(true);
        auto new_scaling = torch::log(torch::from_blob(scales.data(), {static_cast<long>(positions.size()), 1}, pointType)).repeat({1, 3}).to(torch::kCUDA, true).set_requires_grad(true);
        auto new_rotation = torch::zeros({new_xyz.size(0), 4}).index_put_({Slice(), 0}, 1).to(torch::kCUDA, true).set_requires_grad(true);
        auto new_opacity = inverse_sigmoid(0.5 * torch::ones({new_xyz.size(0), 1})).to(torch::kCUDA, true).set_requires_grad(true);

        // colors
        auto fused_color = RGB2SH(torch::from_blob(colors.data(), {static_cast<long>(colors.size()), 3}, colorType).to(pointType) / 255.f).to(torch::kCUDA);

        // features
        auto features = torch::zeros({fused_color.size(0), 3, static_cast<long>(std::pow((_sh_degree + 1), 2))}).to(torch::kCUDA);
        features.index_put_({Slice(), Slice(torch::indexing::None, 3), 0}, fused_color);
        auto new_features_dc = features.index({Slice(), Slice(), Slice(0, 1)}).transpose(1, 2).contiguous().set_requires_grad(true);
        auto new_features_rest = features.index({Slice(), Slice(), Slice(1, torch::indexing::None)}).transpose(1, 2).contiguous().set_requires_grad(true);

        cat_tensors_to_optimizer(optimizer.get(), new_xyz, _xyz, 0);
        cat_tensors_to_optimizer(optimizer.get(), new_features_dc, _features_dc, 1);
        cat_tensors_to_optimizer(optimizer.get(), new_features_rest, _features_rest, 2);
        cat_tensors_to_optimizer(optimizer.get(), new_scaling, _scaling, 3);
        cat_tensors_to_optimizer(optimizer.get(), new_rotation, _rotation, 4);
        cat_tensors_to_optimizer(optimizer.get(), new_opacity, _opacity, 5);
    }
}


void GaussianModel::training_setup()
{
    std::vector<torch::optim::OptimizerParamGroup> optimizer_params_groups;
    optimizer_params_groups.reserve(6);

    optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_xyz}, std::make_unique<torch::optim::AdamOptions>(optimParams.position_lr)));
    optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_features_dc}, std::make_unique<torch::optim::AdamOptions>(optimParams.feature_lr)));
    optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_features_rest}, std::make_unique<torch::optim::AdamOptions>(optimParams.feature_lr / 20.)));
    optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_scaling}, std::make_unique<torch::optim::AdamOptions>(optimParams.scaling_lr)));
    optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_rotation}, std::make_unique<torch::optim::AdamOptions>(optimParams.rotation_lr)));
    optimizer_params_groups.push_back(torch::optim::OptimizerParamGroup({_opacity}, std::make_unique<torch::optim::AdamOptions>(optimParams.opacity_lr)));

    static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[0].options()).eps(1e-15);
    static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[1].options()).eps(1e-15);
    static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[2].options()).eps(1e-15);
    static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[3].options()).eps(1e-15);
    static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[4].options()).eps(1e-15);
    static_cast<torch::optim::AdamOptions&>(optimizer_params_groups[5].options()).eps(1e-15);

    optimizer = std::make_unique<torch::optim::Adam>(optimizer_params_groups, torch::optim::AdamOptions(0.f).eps(1e-15));
}


std::vector<std::string> GaussianModel::construct_list_of_attributes()
{
    std::vector<std::string> attributes = {"x", "y", "z", "nx", "ny", "nz"};

    for (int i = 0; i < _features_dc.size(1) * _features_dc.size(2); ++i)
        attributes.push_back("f_dc_" + std::to_string(i));

    for (int i = 0; i < _features_rest.size(1) * _features_rest.size(2); ++i)
        attributes.push_back("f_rest_" + std::to_string(i));

    attributes.emplace_back("opacity");

    for (int i = 0; i < _scaling.size(1); ++i)
        attributes.push_back("scale_" + std::to_string(i));

    for (int i = 0; i < _rotation.size(1); ++i)
        attributes.push_back("rot_" + std::to_string(i));

    return attributes;
}


void GaussianModel::Save_ply(const std::filesystem::path& file_path, int frame, bool isLastFrame)
{
    std::cout << "Saving at " << std::to_string(frame) << " frames\n";
    auto folder = file_path / ("iteration_" + std::to_string(frame));
    std::filesystem::create_directories(folder);

    auto xyz = _xyz.cpu().contiguous();
    auto normals = torch::zeros_like(xyz);
    auto f_dc = _features_dc.transpose(1, 2).flatten(1).cpu().contiguous();
    auto f_rest = _features_rest.transpose(1, 2).flatten(1).cpu().contiguous();
    auto opacities = _opacity.cpu();
    auto scale = _scaling.cpu();
    auto rotation = _rotation.cpu();

    std::vector<torch::Tensor> tensor_attributes = {xyz.clone(), normals.clone(), f_dc.clone(), f_rest.clone(), opacities.clone(), scale.clone(), rotation.clone()};
    auto attributes = construct_list_of_attributes();
    std::thread t = std::thread([folder, tensor_attributes, attributes]() { Write_output_ply(folder / "point_cloud.ply", tensor_attributes, attributes); });

    if (isLastFrame) {
        t.join();
    }
    else {
        t.detach();
    }
}


void cat_tensors_to_optimizer(torch::optim::Adam* optimizer, torch::Tensor& extension_tensor, torch::Tensor& old_tensor, int param_position)
{
    auto adamParamStates = std::make_unique<torch::optim::AdamParamState>(
        static_cast<torch::optim::AdamParamState&>(*optimizer->state()[c10::guts::to_string(optimizer->param_groups()[param_position].params()[0].unsafeGetTensorImpl())]));
    optimizer->state().erase(c10::guts::to_string(optimizer->param_groups()[param_position].params()[0].unsafeGetTensorImpl()));

    adamParamStates->exp_avg(torch::cat({adamParamStates->exp_avg(), torch::zeros_like(extension_tensor)}, 0));
    adamParamStates->exp_avg_sq(torch::cat({adamParamStates->exp_avg_sq(), torch::zeros_like(extension_tensor)}, 0));

    optimizer->param_groups()[param_position].params()[0] = torch::cat({old_tensor, extension_tensor}, 0).set_requires_grad(true);
    old_tensor = optimizer->param_groups()[param_position].params()[0];

    optimizer->state()[c10::guts::to_string(optimizer->param_groups()[param_position].params()[0].unsafeGetTensorImpl())] = std::move(adamParamStates);
}

} // namespace gs
