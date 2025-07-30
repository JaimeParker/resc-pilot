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

#include <ros/ros.h>
#include <thread>

#include "plan_admin/ctrl_point_gen.h"
#include "plan_admin/motion_gen.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "plan_admin_node");
    ros::NodeHandle nh("~");

    CtrlPointGen kino_search;
    kino_search.init(nh);

    MotionGen rl_optimize;
    rl_optimize.init(nh);

    std::thread search_thread([&]() {
        kino_search.runPublishingLoop();
    });
    std::thread rl_thread([&]() {
        rl_optimize.rlMotion();
    });

    search_thread.join();
    rl_thread.join();

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
