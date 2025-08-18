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
 * Created by Zhaohong Liu on 24-9-18.
*/
#include <ros/ros.h>

#include "rviz_utils/PoseVisualization.h"

using Ptr = std::unique_ptr<MarkerHandler>;
using RP = RvizParams;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_visualization_node");
    ros::NodeHandle nh("~");

    Ptr pose_marker_handler_ptr = std::make_unique<PoseMarkerHandler>(nh, RP::world_frame_id_);
    pose_marker_handler_ptr->init();

    auto rate = ros::Rate(RP::rviz_frequency_);

    while (ros::ok()) {
        pose_marker_handler_ptr->publish();

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
