//
// Created by Zhaohong Liu on 24-11-7.
//

#include <ros/ros.h>
#include "px4_utils_land/PX4CtrlFSM.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "px4_ctrl_fsm_node");
    ros::NodeHandle nh("~");

    PX4CtrlFSM px4_ctrl_fsm;
    px4_ctrl_fsm.init(nh);

    ros::Duration(1.0).sleep();
    ros::spin();  // must call

    return 0;
}