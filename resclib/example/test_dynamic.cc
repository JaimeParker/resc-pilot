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
 * Created by Zhaohong Liu on 24-9-11.
*/

#include <iostream>
#include <chrono>

#include "QuadrotorDynamics.h"

int main() {
    QuadrotorDynamics dyn;
    dyn.initDrone("iris");
    float ca_min = 0.1;
    float ca_max = 0.9;
    dyn.resetCtrlAllocRange(ca_min, ca_max);

    State state_cur = State::Zero();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d attitude = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
    state_cur << position, velocity, attitude, body_rate;

    Eigen::Vector3d body_rate_setpoint(0, 0.0, 0);
    Eigen::Vector4d motor_thrusts = Eigen::Vector4d::Ones();
    double G = RP::G * 0.535;
    motor_thrusts.array() *= G / 4;
    double thrust = 9.8 * 0.535;

    State state_next;
    dyn.setSysCtrlAlloc(true);
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 50; i++) {
        state_next = dyn.run(state_cur, body_rate_setpoint, motor_thrusts, thrust);
        state_cur = state_next;
        motor_thrusts = dyn.getFinalMotorThrusts();
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double average_time = static_cast<double>(duration.count()) / 1e3 / 1e3;
    std::cout << "Time used: " << average_time << " m seconds" << std::endl;

    position = state_next.segment(0, 3);
    velocity = state_next.segment(3, 3);
    attitude = state_next.segment(6, 3);
    body_rate = state_next.segment(9, 3);
    auto final_motor_thrusts = dyn.getFinalMotorThrusts();

    std::cout << "position: " << position.transpose() << std::endl;
    std::cout << "velocity: " << velocity.transpose() << std::endl;
    std::cout << "attitude: " << attitude.transpose() << std::endl;
    std::cout << "body_rate: " << body_rate.transpose() << std::endl;
    std::cout << "final_motor_thrusts: " << final_motor_thrusts.transpose() << std::endl;



    dyn.resetDomainRandomization();

    return 0;
}