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
 * Created by Zhaohong Liu on 2024/9/3.
 * Reference: https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/rate_control
*/

#ifndef RATECONTROL_H
#define RATECONTROL_H

#include <Eigen/Eigen>
#include <memory>
#include <iostream>

#include "Params.h"
#include "drone/DroneBase.h"

class RateControl {
private:
    /* MC Rate Control */
    Eigen::Matrix3d rate_p_gain_ = Eigen::Matrix3d::Identity();

    /* Mixer */
    Eigen::Matrix4d alloc_mat_ = Eigen::Matrix4d::Ones();
    Eigen::Matrix4d inv_alloc_mat_ = Eigen::Matrix4d::Ones();

    /* Motor Model */
    double motor_thrust_min_ = -1.0;
    double motor_thrust_max_ = -1.0;

    /* drone */
    std::shared_ptr<DroneBase> drone_ptr_;

public:
    explicit RateControl(std::shared_ptr<DroneBase> & drone);
    void setThrustRange();
    [[nodiscard]] Eigen::Vector3d update(const Eigen::Vector3d &body_rate,
                                         const Eigen::Vector3d &body_rate_setpoint,
                                         double collective_thrust) const;
    Eigen::Vector4d mixer(Eigen::Vector3d &torque_body, double thrust) const;
    void clampThrusts(Eigen::Vector4d& motor_thrusts) const;
    [[nodiscard]] Eigen::Matrix4d getAllocMat() const;
    void checkParams() const;

public:
    using Ptr = std::unique_ptr<RateControl>;
};



#endif //RATECONTROL_H
