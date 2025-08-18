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
 * Created by Zhaohong Liu on 24-11-18.
*/

#include "ControlAllocator.h"

ControlAllocator::ControlAllocator(std::shared_ptr<DroneBase> & drone) : drone_ptr_(drone) {
    updateDroneParams();
    mixing_output_.setDrone(drone);
}

void ControlAllocator::updateDroneParams() {
    rotor_positions_ << static_cast<float>(drone_ptr_->arm_x_), static_cast<float>(drone_ptr_->arm_y_front_), 0,
                        -static_cast<float>(drone_ptr_->arm_x_), -static_cast<float>(drone_ptr_->arm_y_rear_), 0,
                        static_cast<float>(drone_ptr_->arm_x_), -static_cast<float>(drone_ptr_->arm_y_front_), 0,
                        -static_cast<float>(drone_ptr_->arm_x_), static_cast<float>(drone_ptr_->arm_y_rear_), 0;

    moment_constants_ << static_cast<float>(drone_ptr_->torque_constant_),
                         static_cast<float>(drone_ptr_->torque_constant_),
                         -static_cast<float>(drone_ptr_->torque_constant_),
                         -static_cast<float>(drone_ptr_->torque_constant_);

    thrust_coefficients_ << static_cast<float>(drone_ptr_->thrust_coefficient_),
                            static_cast<float>(drone_ptr_->thrust_coefficient_),
                            static_cast<float>(drone_ptr_->thrust_coefficient_),
                            static_cast<float>(drone_ptr_->thrust_coefficient_);

    updateEffectivenessMix();
}

void ControlAllocator::updateEffectivenessMix() {
    effectiveness_.setZero();

    for (int i = 0; i < rotor_positions_.rows(); i++) {
        auto position = rotor_positions_.row(i).transpose();
        auto ct = thrust_coefficients_(i, 0);
        auto km = moment_constants_(i, 0);

        Eigen::Vector3f thrust = ct * axis_upward_;
        Eigen::Vector3f moment = ct * position.cross(axis_upward_) - ct * km * axis_upward_;

        effectiveness_.block<3, 1>(0, i) = moment;  // Rows 0-2: moments
        effectiveness_.block<3, 1>(3, i) = thrust;  // Rows 3-5: thrust
    }

    if (Convertor::getInv(effectiveness_, mix_)) {
        normalizeMix();
    } else {
        throw std::runtime_error("Cannot invert effectiveness matrix");
    }
}

void ControlAllocator::normalizeMix() {
    Eigen::Matrix<float, NUM_AXES, 1> control_allocation_scale;
    control_allocation_scale.setOnes();

    int num_non_zero_roll_torque = 0;
    int num_non_zero_pitch_torque = 0;

    for (int i = 0; i < NUM_ACTUATORS; ++i) {
        if (std::abs(mix_(i, 0)) > 1e-3f) {
            ++num_non_zero_roll_torque;
        }
        if (std::abs(mix_(i, 1)) > 1e-3f) {
            ++num_non_zero_pitch_torque;
        }
    }


    float roll_norm_scale = 1.0f;
    if (num_non_zero_roll_torque > 0) {
        roll_norm_scale = std::sqrt(mix_.col(0).squaredNorm() / (static_cast<float>(num_non_zero_roll_torque) / 2.0f));
    }

    float pitch_norm_scale = 1.0f;
    if (num_non_zero_pitch_torque > 0) {
        pitch_norm_scale = std::sqrt(mix_.col(1).squaredNorm() / (static_cast<float>(num_non_zero_pitch_torque) / 2.0f));
    }

    // same scale for roll and pitch
    control_allocation_scale(0) = std::max(roll_norm_scale, pitch_norm_scale);
    control_allocation_scale(1) = control_allocation_scale(0);
    // set yaw separately
    control_allocation_scale(2) = mix_.col(2).cwiseAbs().maxCoeff();

    // thrust
    control_allocation_scale(5) = 1.0f;
    for (int axis_idx = 2; axis_idx >= 0; --axis_idx) {
        int num_non_zero_thrust = 0;
        float norm_sum = 0.0f;

        for (int i = 0; i < NUM_ACTUATORS; ++i) {
            float norm = std::abs(mix_(i, 3 + axis_idx));
            norm_sum += norm;

            if (norm > FLT_EPSILON) {
                ++num_non_zero_thrust;
            }
        }

        if (num_non_zero_thrust > 0) {
            control_allocation_scale(3 + axis_idx) = norm_sum / static_cast<float>(num_non_zero_thrust);
        } else {
            control_allocation_scale(3 + axis_idx) = control_allocation_scale(5);  // THRUST_Z
        }
    }

    // Normalize the mix matrix
    if (control_allocation_scale(0) > FLT_EPSILON) {
        mix_.col(0) /= control_allocation_scale(0);
        mix_.col(1) /= control_allocation_scale(1);
    }

    if (control_allocation_scale(2) > FLT_EPSILON) {
        mix_.col(2) /= control_allocation_scale(2);
    }

    for (int i = 3; i <= 5; ++i) {
        if (control_allocation_scale(i) > FLT_EPSILON) {
            mix_.col(i) /= control_allocation_scale(i);
        }
    }

    // Set all small elements to 0
    for (int i = 0; i < NUM_ACTUATORS; ++i) {
        for (int j = 0; j < NUM_AXES; ++j) {
            if (std::abs(mix_(i, j)) < 1e-3f) {
                mix_(i, j) = 0.0f;
            }
        }
    }
}

void ControlAllocator::setControlSetpoint(const Eigen::Vector3d &torque_sp, const double & thrust_sp) {
    // FLU torque to NED
    control_sp_(0) = static_cast<float>(torque_sp.x());
    control_sp_(1) = static_cast<float>(-torque_sp.y());
    control_sp_(2) = static_cast<float>(-torque_sp.z());
    control_sp_(3) = thrust_x_;
    control_sp_(4) = thrust_y_;
    // rescale thrust to [0, 1] and reverse
    auto thrust_sp_rescaled = static_cast<float>(drone_ptr_->rescaleThrust(thrust_sp));
    control_sp_(5) = -thrust_sp_rescaled;
}

void ControlAllocator::pseudoInverseAllocate() {
    actuator_sp_ = mix_ * control_sp_;
    clipActuatorSetpoint();
    mixing_output_.updateActuatorSetpointValues(actuator_sp_);
}

void ControlAllocator::clipActuatorSetpoint() {
    actuator_sp_ = actuator_sp_.cwiseMax(drone_ptr_->ca_min_).cwiseMin(drone_ptr_->ca_max_);
}

void MixingOutput::updateActuatorSetpointValues(const Eigen::Matrix<float, NUM_ACTUATORS, 1> &actuator_sp) {
    actuator_motors_ = actuator_sp;

    if (drone_ptr_->thr_mdl_fac_ > 0.f && drone_ptr_->thr_mdl_fac_ < 1.f) {
        // thrust factor
        //  rel_thrust = factor * x^2 + (1-factor) * x,
        const float a = drone_ptr_->thr_mdl_fac_;
        const float b = 1.f - a;

        // don't recompute for all values (ax^2+bx+c=0)
        const float tmp1 = b / (2.f * a);
        const float tmp2 = b * b / (4.f * a * a);

        for (int i = 0; i < NUM_ACTUATORS; ++i) {
            float control = actuator_motors_(i);

            if (control > 0.f) {
                actuator_motors_(i) = -tmp1 + sqrtf(tmp2 + (control / a));
            } else if (control < -0.f) {
                actuator_motors_(i) = tmp1 - sqrtf(tmp2 - (control / a));
            } else {
                actuator_motors_(i) = 0.f;
            }
        }
    }

    for (int i = 0; i < NUM_ACTUATORS; ++i) {
        // remap from [0, 1] to [-1, 1]
        actuator_motors_(i) = actuator_motors_(i) * 2.f - 1.f;
    }

    outputLimitCalcSingle();
}

void MixingOutput::outputLimitCalcSingle() {
    for (int i = 0; i < NUM_ACTUATORS; i++) {
        actuator_outputs_(i) = pwm_sum_half_ + pwm_diff_half_ * actuator_motors_(i);
    }
    actuator_outputs_ = actuator_outputs_.cwiseMax(pwm_min_).cwiseMin(pwm_max_);

    for (int i = 0; i < 4; i++) {
        double pwm = actuator_outputs_(i);
        motor_thrusts_(i) = drone_ptr_->pwm2Thrust(pwm);
    }

    reorderThrusts();
}

void MixingOutput::reorderThrusts() {
    // PX4 actuators config
    // 3   1
    //  \ /
    //   X
    //  / \
    // 2   4
    // local NED frame, x forward, y right, z down

    // In my RL setting (应该尽早合并)
    // 4   1
    //  \ /
    //   X
    //  / \
    // 3   2

    // PX4 order to my RL setting
    double thrusts[4];
    thrusts[0] = motor_thrusts_(0);
    thrusts[1] = motor_thrusts_(1);
    thrusts[2] = motor_thrusts_(2);
    thrusts[3] = motor_thrusts_(3);

    motor_thrusts_(0) = thrusts[0];
    motor_thrusts_(1) = thrusts[3];
    motor_thrusts_(2) = thrusts[1];
    motor_thrusts_(3) = thrusts[2];
}
