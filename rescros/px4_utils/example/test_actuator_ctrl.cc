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
 * Created by Zhaohong Liu on 24-9-23.
*/

#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_actuator_ctrl_node");
    ros::NodeHandle nh;

    ros::Publisher actuator_ctrl_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
    mavros_msgs::ActuatorControl actuator_ctrl;

    ros::Rate rate(50.0);
    while (ros::ok()) {
        actuator_ctrl.header.stamp = ros::Time::now();
        actuator_ctrl.group_mix = mavros_msgs::ActuatorControl::PX4_MIX_FLIGHT_CONTROL;
        actuator_ctrl.controls = {0, 0, 0, 0, 0, 0, 0, 0};

        actuator_ctrl_pub.publish(actuator_ctrl);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}