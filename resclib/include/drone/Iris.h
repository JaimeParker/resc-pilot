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
 * Created by Zhaohong Liu on 24-10-28.
*/

#ifndef RESCLIB_IRIS_H
#define RESCLIB_IRIS_H

#include "DroneBase.h"

class Iris : public DroneBase {
private:
    const double motor_constant_ = 5.84e-6;
public:
    Iris() {
        mass_ = 0.535;
        arm_x_ = 0.13;
        arm_y_front_ = 0.22;
        arm_y_rear_ = 0.2;

        ixx_ = 0.029125;
        iyy_ = 0.029125;
        izz_ = 0.055225;
        torque_constant_ = 0.12;
        thrust_coefficient_ = 6.925;

        mc_roll_rate_p_ = 0.15;
        mc_pitch_rate_p_ = 0.15;
        mc_yaw_rate_p_ = 0.2;

        rotor_rise_time_ = 0.001;
    }

    double pwm2Thrust(double &pwm) override {
        return std::pow(10 * pwm - 9000, 2) * motor_constant_ / 1000 * g_;
        // from px4_sitl gazebo iris motor model
        // rot_vel = rescaled_thrust * ROTOR_INPUT_SCALING + ROTOR_POSITION_ARMED
        // real_velocity = rot_vel * ROTOR_VEL_SLOWDOWN_SIM
        // thrust_g = std::pow(real_velocity, 2) * MOTOR_CONSTANT
        // thrust (newton) = thrust_g / 1000 * 9.8
        // ROTOR_INPUT_SCALING = 1000;
        // ROTOR_POSITION_ARMED = 100;
        // ROTOR_VEL_SLOWDOWN_SIM = 10;
        // MOTOR_CONSTANT = 5.84e-6;
        // T = (1e4 * t + 1e3)^2 * 5.84e-6 / 1000 * 9.8
    }

    double rescaleThrust(const double &thrust) override {
        double single_thrust = thrust / 4;
        return (sqrt(single_thrust / g_ / motor_constant_ * 1e3) - 1e3) / 1e4;
    }
};

#endif //RESCLIB_IRIS_H
