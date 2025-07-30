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

#ifndef RESCLIB_DRONEBASE_H
#define RESCLIB_DRONEBASE_H

#include <cmath>

class DroneBase {
public:
    DroneBase() = default;
    virtual ~DroneBase() = default;
    virtual double pwm2Thrust(double & pwm) = 0;
    virtual void resetPwmRange(double & pwm_min, double & pwm_max) {
        pwm_min_ = pwm_min;
        pwm_max_ = pwm_max;
    }
    virtual double mavrosAttTarThrust2Pwm(double & thrust) {
        return thrust * (pwm_max_ - pwm_min_) + pwm_min_;
    }
    virtual void resetCARange(float & ca_min, float & ca_max) {
        ca_min_ = ca_min;
        ca_max_ = ca_max;
    }

    /**
     * @brief Rescale collective thrust to mavros thrust in [0, 1]
     * @param thrust collective thrust in newton
     * @return rescaled thrust in [0, 1] that can be used in mavros attitude target message
     */
    virtual double rescaleThrust(const double & thrust) = 0;
public:
    /* size and mass */
    double mass_ = -1.0;
    double arm_x_ = -1.0;
    double arm_y_front_ = -1.0;
    double arm_y_rear_ = -1.0;

    /* motor and inertia */
    double ixx_ = -1.0;
    double iyy_ = -1.0;
    double izz_ = -1.0;
    double torque_constant_ = -1.0;  // km
    double thrust_coefficient_ = -1.0;  // ct

    /* multi copter rate control */
    double mc_roll_rate_p_ = -1.0;
    double mc_pitch_rate_p_ = -1.0;
    double mc_yaw_rate_p_ = -1.0;

    /* rl params */
    double ct_g_min_ = 0.3;
    double ct_g_max_ = 1.7;
    double g_ = 9.8;

    /* px4 and qgc */
    double pwm_min_ = 1000;
    double pwm_max_ = 2000;
    float ca_min_ = 0.0;
    float ca_max_ = 1.0;
    float thr_mdl_fac_ = 0.0;

    /* rotor */
    double rotor_rise_time_ = 100.0;

    /* utils */
    static constexpr double deg2rad_ = M_PI / 180.0;
};

#endif //RESCLIB_DRONEBASE_H
