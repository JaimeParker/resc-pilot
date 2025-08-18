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
 * Created by Zhaohong Liu on 25-5-21.
*/

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <mavros_msgs/CommandBool.h>

// Function to set terminal attributes for non-blocking input
void setNonBlockingInput() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~(ICANON | ECHO); // Turn off canonical mode and echo
    ttystate.c_cc[VMIN] = 0;
    ttystate.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    
    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

// Restore normal terminal operation
void restoreTerminal() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag |= ICANON | ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "land_command_publisher");
    ros::NodeHandle nh("~");
    
    std::string land_topic;
    if (!nh.getParam("land_topic", land_topic)) {
        land_topic = "/trigger_landing";
        ROS_WARN("[Land Command]: Using default land topic: %s", land_topic.c_str());
    }
    
    ros::Publisher land_pub = nh.advertise<std_msgs::Bool>(land_topic, 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    std_msgs::Bool land_msg;
    land_msg.data = true;
    
    setNonBlockingInput(); // Configure terminal
    
    // Print usage info
    std::cout << "=========================" << std::endl;
    std::cout << "PX4 FSM Command Interface" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  l - Trigger landing" << std::endl;
    std::cout << "  q - Quit" << std::endl;
    std::cout << "=========================" << std::endl;
    
    ros::Rate rate(10);
    bool key_pressed = false;
    
    while (ros::ok()) {
        int key = -1;
        
        // Check if there's input available
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            key = c;
            key_pressed = true;
        }
        
        // Process key input
        if (key_pressed) {
            std::cout << "\r\033[K"; // Clear the current line
            
            switch (key) {
                case 'l': 
                case 'L':
                    std::cout << "\033[1;32m[PX4 FSM USER INPUT]: l - Sending landing command...\033[0m" << std::endl;
                    land_pub.publish(land_msg);
                    break;
                    
                case 'q':
                case 'Q':
                    std::cout << "\033[1;36m[PX4 FSM USER INPUT]: q - Exiting...\033[0m" << std::endl;
                    restoreTerminal();
                    return 0;
                    
                default:
                    std::cout << "\033[1;37m[PX4 FSM USER INPUT]: '" << static_cast<char>(key) 
                              << "' - Unknown command\033[0m" << std::endl;
                    break;
            }
            
            key_pressed = false;
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    restoreTerminal(); // Restore terminal settings when exiting
    return 0;
}