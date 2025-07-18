//
// Created by Zhaohong Liu on 2024/9/23.
//

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