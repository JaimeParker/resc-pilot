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

#ifndef RESCLIB_CONTROLALLOCATOR_H
#define RESCLIB_CONTROLALLOCATOR_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <memory>
#include <cfloat>

#include "drone/DroneBase.h"
#include "Convertor.h"

static constexpr int NUM_ACTUATORS = 16;
static constexpr int NUM_AXES = 6;

class MixingOutput {
public:
    void setDrone(std::shared_ptr<DroneBase> & drone) {
        drone_ptr_ = drone;
        pwm_min_ = static_cast<float>(drone_ptr_->pwm_min_);
        pwm_max_ = static_cast<float>(drone_ptr_->pwm_max_);
        pwm_sum_half_ = (pwm_min_ + pwm_max_) / 2;
        pwm_diff_half_ = (pwm_max_ - pwm_min_) / 2;
    }
    void updateActuatorSetpointValues(const Eigen::Matrix<float, NUM_ACTUATORS, 1> & actuator_sp);
    void outputLimitCalcSingle();
    void reorderThrusts();
public:
    Eigen::Vector4d motor_thrusts_;
private:
    float pwm_min_, pwm_max_, pwm_sum_half_, pwm_diff_half_;
    Eigen::Matrix<float, NUM_ACTUATORS, 1> actuator_motors_;
    Eigen::Matrix<float, NUM_ACTUATORS, 1> actuator_outputs_;
    std::shared_ptr<DroneBase> drone_ptr_;
};

class ControlAllocator {
private:
    static constexpr float thrust_x_ = 0.0;
    static constexpr float thrust_y_ = 0.0;

    const Eigen::Vector3f axis_upward_ = Eigen::Vector3f(0, 0, -1.0);

    Eigen::Matrix<float, 4, 3> rotor_positions_;
    Eigen::Matrix<float, 4, 1> moment_constants_;
    Eigen::Matrix<float, 4, 1> thrust_coefficients_;

    Eigen::Matrix<float, NUM_AXES, NUM_ACTUATORS> effectiveness_;
    Eigen::Matrix<float, NUM_ACTUATORS, NUM_AXES> mix_;
    Eigen::Matrix<float, NUM_AXES, 1> control_sp_;
    Eigen::Matrix<float, NUM_ACTUATORS, 1> actuator_sp_;
    std::shared_ptr<DroneBase> drone_ptr_;

    MixingOutput mixing_output_;
public:
    explicit ControlAllocator(std::shared_ptr<DroneBase> & drone);
    void updateDroneParams();
    void updateEffectivenessMix();
    void normalizeMix();

    /**
     * @brief Set the control setpoint, torque and thrust
     * @param torque_sp
     * @param thrust_sp real thrust in newton, not normalized
     */
    void setControlSetpoint(const Eigen::Vector3d & torque_sp, const double & thrust_sp);
    void pseudoInverseAllocate();
    void clipActuatorSetpoint();

    [[nodiscard]] const Eigen::Matrix<float, NUM_AXES, NUM_ACTUATORS> & getEffectiveness() const { return effectiveness_; }
    [[nodiscard]] const Eigen::Matrix<float, NUM_ACTUATORS, NUM_AXES> & getMix() const { return mix_; }
    [[nodiscard]] const Eigen::Vector4d & getMotorThrusts() const { return mixing_output_.motor_thrusts_; }
};

#endif //RESCLIB_CONTROLALLOCATOR_H
