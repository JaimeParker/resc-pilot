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
 * Created by Zhaohong Liu on 24-9-3.
*/

#include "RateControl.h"

RateControl::RateControl(std::shared_ptr<DroneBase> & drone) {
    drone_ptr_ = drone;
    rate_p_gain_(0, 0) = drone_ptr_->mc_roll_rate_p_;
    rate_p_gain_(1, 1) = drone_ptr_->mc_pitch_rate_p_;
    rate_p_gain_(2, 2) = drone_ptr_->mc_yaw_rate_p_;

    alloc_mat_(1, 0) = -drone_ptr_->arm_y_front_;
    alloc_mat_(1, 1) = -drone_ptr_->arm_y_rear_;
    alloc_mat_(1, 2) = drone_ptr_->arm_y_rear_;
    alloc_mat_(1, 3) = drone_ptr_->arm_y_front_;

    alloc_mat_(2, 0) = -drone_ptr_->arm_x_;
    alloc_mat_(2, 1) = drone_ptr_->arm_x_;
    alloc_mat_(2, 2) = drone_ptr_->arm_x_;
    alloc_mat_(2, 3) = -drone_ptr_->arm_x_;

    alloc_mat_(3, 0) = -drone_ptr_->torque_constant_;
    alloc_mat_(3, 1) = drone_ptr_->torque_constant_;
    alloc_mat_(3, 2) = -drone_ptr_->torque_constant_;
    alloc_mat_(3, 3) = drone_ptr_->torque_constant_;

    inv_alloc_mat_ = alloc_mat_.inverse();

    setThrustRange();
}

Eigen::Vector3d RateControl::update(const Eigen::Vector3d &body_rate,
                                    const Eigen::Vector3d &body_rate_setpoint,
                                    const double collective_thrust) const {
    const auto rate_error = body_rate_setpoint - body_rate;
    // https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/rate_control/rate_control.cpp#L78
    // PX4 version, if using P only
    Eigen::Vector3d torque_req = rate_p_gain_ * rate_error;

    return torque_req;
}

Eigen::Vector4d RateControl::mixer(Eigen::Vector3d &torque_body, double thrust) const {
    const Eigen::Vector4d ctrl_unit(thrust, torque_body[0], torque_body[1], torque_body[2]);
    Eigen::Vector4d motor_thrusts = inv_alloc_mat_ * ctrl_unit;

    clampThrusts(motor_thrusts);

    return motor_thrusts;
}

void RateControl::clampThrusts(Eigen::Vector4d &motor_thrusts) const {
    std::for_each(motor_thrusts.data(), motor_thrusts.data() + motor_thrusts.size(),
                  [this](double &thrust) {
                      thrust = std::clamp(thrust, motor_thrust_min_, motor_thrust_max_);
                  });
}

void RateControl::setThrustRange() {
    motor_thrust_min_ = drone_ptr_->pwm2Thrust(drone_ptr_->pwm_min_);
    motor_thrust_max_ = drone_ptr_->pwm2Thrust(drone_ptr_->pwm_max_);
}

Eigen::Matrix4d RateControl::getAllocMat() const {
    return alloc_mat_;
}

void RateControl::checkParams() const {
    if (motor_thrust_min_ < 0 || motor_thrust_max_ < 0) {
        throw std::runtime_error("Motor thrust min or max is not set.");
    }

    if (alloc_mat_.isZero()) {
        throw std::runtime_error("Allocation matrix is not set.");
    }

    if (inv_alloc_mat_.isZero()) {
        throw std::runtime_error("Inverse allocation matrix is not set.");
    }

    if (rate_p_gain_.isZero()) {
        throw std::runtime_error("Rate P gain is not set.");
    }

    std::cout << "P gain for roll, pitch, yaw: " << rate_p_gain_.diagonal().transpose() << std::endl;
    std::cout << "Motor thrust min (single rotor): " << motor_thrust_min_ << std::endl;
    std::cout << "Motor thrust max (single rotor): " << motor_thrust_max_ << std::endl;

    // cout the allocation matrix
    std::cout << "Allocation matrix: " << std::endl;
    std::cout << alloc_mat_ << std::endl;
}
