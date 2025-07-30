//
// Created by Zhaohong Liu on 25-5-21.
//

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "land_command_publisher");
    ros::NodeHandle nh("~");
    
    std::string land_topic;
    if (!nh.getParam("land_topic", land_topic)) {
        land_topic = "/trigger_landing";
        ROS_WARN("[Land Command]: Using default land topic: %s", land_topic.c_str());
    }
    
    ros::Publisher land_pub = nh.advertise<std_msgs::Bool>(land_topic, 10);
    
    std_msgs::Bool land_msg;
    land_msg.data = true;
    
    ROS_INFO("[Land Command]: Node initialized. Press 'l' to trigger landing, 'q' to quit.");
    
    ros::Rate rate(10);
    while (ros::ok()) {
        int key = -1;
        
        // read keyboard input
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv = {0, 1000};  // 1ms timeout
        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0) {
            key = getchar();
        }
        
        // process key input
        if (key == 'l' || key == 'L') {
            std::cout << "[PX4 FSM USER]: Sending landing command..." << std::endl;
            land_pub.publish(land_msg);
        } else if (key == 'q' || key == 'Q') {
            ROS_INFO("[PX4 FSM USER]: Exiting...");
            break;
        } else if (key == 'a' || key == 'A') {
            
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}