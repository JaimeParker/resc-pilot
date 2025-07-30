/**
 * This file is part of Resc-Pilot.
 *
 * Copyright 2025 Zhaohong Liu, IRMV Lab, Shanghai Jiao Tong University, <https://www.sjtu.edu.cn/>
 * Developed by Zhaohong Liu <zhliu25 at outlook dot com>, <jaimefriedhelmzhao at gmail dot com>
 * for more information see <https://github.com/JaimeParker/resc-pilot>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Resc-Pilot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Resc-Pilot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Resc-Pilot. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Created by Zhaohong Liu on 24-5-9.
*/

#include <backend_optimizer/rl_motion.h>

RLMotion::RLMotion() {
    pos_ = Eigen::Vector3d::Zero();
    vel_ = Eigen::Vector3d::Zero();
    acc_ = Eigen::Vector3d::Zero();
    att_ = Eigen::Vector3d::Zero();
    body_rate_ = Eigen::Vector3d::Zero();
    num_obs_ = 0;
}

RLMotion::~RLMotion() {
    std::cout<< "backend_optimizer/RLMotion is destructed" << std::endl;
}

std::vector<float> RLMotion::getObservation() {
    Eigen::Vector3d pos2wpt0 = wpt_pair_.first - pos_;
    Eigen::Vector3d pos2wpt1 = wpt_pair_.second - pos_;
    Eigen::Vector3d wpt0_2_wpt1 = wpt_pair_.second - wpt_pair_.first;

    Eigen::Matrix3d rot_mat = getRotMat(att_);
    auto xb_w = rot_mat * Eigen::Vector3d::UnitX();

    double dist_pos2wpt0_2d = std::sqrt(pos2wpt0[0] * pos2wpt0[0] + pos2wpt0[1] * pos2wpt0[1]);
    double dist_pos2wpt1_2d = std::sqrt(pos2wpt1[0] * pos2wpt1[0] + pos2wpt1[1] * pos2wpt1[1]);

    // have to convert to float for pytorch
    std::vector<float> values;
    auto append_vector = [&values](const Eigen::Vector3d& vec) {
        values.push_back(static_cast<float>(vec[0]));
        values.push_back(static_cast<float>(vec[1]));
        values.push_back(static_cast<float>(vec[2]));
    };

    if (use_full_observation) {
        values.push_back(static_cast<float>(rela_pos_.x()));
        values.push_back(static_cast<float>(rela_pos_.y()));
    }
    append_vector(vel_);
    append_vector(att_);
    append_vector(body_rate_);

    if (use_full_observation) {
        for (const auto& val : sdf_values_) {
            values.push_back(static_cast<float>(val));
        }
    }
    values.push_back(static_cast<float>(height_top_ - pos_[2]));
    values.push_back(static_cast<float>(pos_[2]));

    if (use_full_observation) {
        for (const auto& val : pseudo_raycast_) {
            values.push_back(static_cast<float>(val));
        }
    }

    append_vector(pos2wpt0);
    append_vector(pos2wpt1);
    append_vector(wpt0_2_wpt1);
    append_vector(xb_w);

    values.push_back(static_cast<float>(dist_pos2wpt0_2d));
    values.push_back(static_cast<float>(dist_pos2wpt1_2d));

    return values;
}

void RLMotion::init() {
    std::string op_path = ros::package::getPath("backend_optimizer");
    std::string model_path = op_path + "/model/" + model_name_;
    try {
        model_ = torch::jit::load(model_path);
    }
    catch (const c10::Error& e) {
        std::cerr << "\033[1;31m[RL model]: error loading the model\033[0m\n" << std::endl;
        std::cerr << "[RL model]: error: " << e.what() << std::endl;
        exit(0);
    }
    ROS_INFO_STREAM("\033[1;34m" << "RL model loaded!" << "\033[0m");
}

Eigen::Matrix3d RLMotion::getRotMat(const Eigen::Vector3d &attitude_euler) {
    double s_phi = std::sin(attitude_euler[0]);
    double c_phi = std::cos(attitude_euler[0]);
    double s_theta = std::sin(attitude_euler[1]);
    double c_theta = std::cos(attitude_euler[1]);
    double s_psi = std::sin(attitude_euler[2]);
    double c_psi = std::cos(attitude_euler[2]);

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << c_theta * c_psi, s_theta * s_phi * c_psi - s_psi * c_phi, s_theta * c_phi * c_psi + s_psi * s_phi,
            c_theta * s_psi, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi,
            -s_theta, s_phi * c_theta, c_phi * c_theta;

    return rotation_matrix;
}

void RLMotion::setAllStates(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                            const Eigen::Vector3d &att, const Eigen::Vector3d &body_rate,
                            WptPair &wpt_pair) {
    pos_ = pos;
    vel_ = vel;
    att_ = att;
    body_rate_ = body_rate;
    wpt_pair_ = wpt_pair;
}

void RLMotion::setAllStates(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &att,
                            const Eigen::Vector3d &body_rate, WptPair &wpt_pair, const Eigen::Vector2d &rela_pos,
                            const std::vector<double> &sdf_values, const std::vector<double> &pseudo_raycast) {
    pos_ = pos;
    vel_ = vel;
    att_ = att;
    body_rate_ = body_rate;
    wpt_pair_ = wpt_pair;

    rela_pos_ = rela_pos;
    sdf_values_ = sdf_values;
    pseudo_raycast_ = pseudo_raycast;

    use_full_observation = true;
}

Eigen::Vector4d RLMotion::forwardModel() {
    std::vector<float> values = getObservation();
    // vector<float> to tensor have to be in the same function with forward
    torch::Tensor obs_tensor_data = torch::from_blob(values.data(), {1, num_obs_});

    std::vector<torch::jit::IValue> obs_tensor;
    obs_tensor.emplace_back(obs_tensor_data);

    // PPO model, Tuple[th.Tensor, th.Tensor, th.Tensor]
    auto output_tuple = model_.forward(obs_tensor).toTuple();
    auto action = output_tuple->elements()[0].toTensor();

    Eigen::Vector4d output_vector;
    output_vector << action[0][0].item().toDouble(), action[0][1].item().toDouble(),
            action[0][2].item().toDouble(), action[0][3].item().toDouble();

    auto rescale = [](double val) {
        if (val > 1.0) {
            return 1.0;
        } else if (val < -1.0) {
            return -1.0;
        } else {
            return val;
        }
    };

    for (int i = 0; i < output_vector.size(); ++i) {
        output_vector[i] = rescale(output_vector[i]);
    }

    obs_tensor.clear();
    return output_vector;
}

void RLMotion::setModelName(const std::string& model_name) {
    if (model_name.empty()) {
        throw std::invalid_argument("Model name cannot be empty");
    }
    model_name_ = model_name;
}

void RLMotion::setNumObs(int num_obs) {
    num_obs_ = num_obs;
}
