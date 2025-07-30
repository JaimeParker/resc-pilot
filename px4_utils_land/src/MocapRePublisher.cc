//
// Created by Zhaohong Liu on 24-9-22.
//

#include <ros/ros.h>
#include "px4_utils_land/MocapProcessor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mocap_rate_publish_node");
    ros::NodeHandle nh;

    MocapProcessor mocap_processor;
    mocap_processor.init(nh);

    auto rate = ros::Rate(200);

    while (ros::ok()) {
        mocap_processor.publishRate();
        mocap_processor.publishThrust();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}