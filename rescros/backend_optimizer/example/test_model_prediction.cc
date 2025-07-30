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
 * Created by Zhaohong Liu on 24-5-10.
*/

#include <Eigen/Eigen>
#include <chrono>

#include <backend_optimizer/rl_motion.h>

int main() {
    RLMotion rl_motion;
    rl_motion.setModelName("2024-08-28_17-14.pt");
    rl_motion.setNumObs(25);
    rl_motion.init();

    RLMotion::Ptr rl_motion_ptr = std::make_unique<RLMotion>();

    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d att = Eigen::Vector3d::Zero();
    Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();

    Eigen::Vector3d wpt0(2, 0, 1);
    Eigen::Vector3d wpt1(3, 0, 1);
    WptPair wpt_pair = std::make_pair(wpt0, wpt1);
    rl_motion.setAllStates(pos, vel, att, body_rate, wpt_pair);
    Eigen::Vector4d action = rl_motion.forwardModel();
    std::cout << "action: " << action.transpose() << std::endl;

    std::cout << "10 times to double check" << std::endl;
    for (int i = 0; i < 10; i++) {
        rl_motion.setAllStates(pos, vel, att, body_rate, wpt_pair);
        action = rl_motion.forwardModel();
        std::cout << "action: " << action.transpose() << std::endl;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; i++) {
        rl_motion.setAllStates(pos, vel, att, body_rate, wpt_pair);
        rl_motion.forwardModel();
    }
    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
    double average_time = static_cast<double>(duration.count()) / 1e6;
    std::cout << "Average time taken by forwardModel: " << average_time << " m seconds" << std::endl;

    return 0;
}