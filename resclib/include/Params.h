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
 * Created by Zhaohong Liu on 24-9-10.
*/

#ifndef RESCLIB_PARAMS_H
#define RESCLIB_PARAMS_H

class RescParams {
public:
    /* RL env params */
    static constexpr double G = 9.8;
    static constexpr double RL_DT = 0.02;
    static constexpr double SIM_DT = 0.01;

    static constexpr double ROTOR_INPUT_SCALING = 1000;
    static constexpr double ROTOR_POSITION_ARMED = 100;
    static constexpr double ROTOR_VEL_SLOWDOWN_SIM = 10;
    static constexpr double ROTOR_RISE_TIME = 0.001;

    /* Dynamic and Kinematic Constraints */
    static constexpr double MAX_ROLL_RATE = 180 * M_PI / 180;
    static constexpr double MAX_PITCH_RATE = 180 * M_PI / 180;
    static constexpr double MAX_YAW_RATE = 60 * M_PI / 180;
    static constexpr double CT_G_MAX = 1.7;
    static constexpr double CT_G_MIN = 0.3;
};

#endif //RESCLIB_PARAMS_H
