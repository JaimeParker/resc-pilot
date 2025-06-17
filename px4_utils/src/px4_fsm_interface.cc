//
// Created by Zhaohong Liu on 25-5-21.
//

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

    std::string editable_mode_topic;
    if (!nh.getParam("editable_mode_topic", editable_mode_topic)) {
        editable_mode_topic = "/editable_mode";
        ROS_WARN("[Land Command]: Using default editable mode topic: %s", editable_mode_topic.c_str());
    }

    std::string rtb_topic;
    if (!nh.getParam("return_to_base_topic", rtb_topic)) {
        rtb_topic = "/return_to_base";
        ROS_WARN("[Land Command]: Using default return to base topic: %s", rtb_topic.c_str());
    }

    std::string arming_topic;
    if (!nh.getParam("arming_topic", arming_topic)) {
        arming_topic = "/trigger_arming";
        ROS_WARN("[Land Command]: Using default arming topic: %s", arming_topic.c_str());
    }
    
    ros::Publisher land_pub = nh.advertise<std_msgs::Bool>(land_topic, 10);
    ros::Publisher editable_mode_pub = nh.advertise<std_msgs::Bool>(editable_mode_topic, 10);
    ros::Publisher rtb_pub = nh.advertise<std_msgs::Bool>(rtb_topic, 10);
    ros::Publisher arm_pub = nh.advertise<std_msgs::Bool>(arming_topic, 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    std_msgs::Bool land_msg;
    land_msg.data = true;

    std_msgs::Bool editable_mode_msg;
    editable_mode_msg.data = false;

    std_msgs::Bool rtb_msg;
    rtb_msg.data = false;

    std_msgs::Bool arm_msg;
    arm_msg.data = false;
    
    setNonBlockingInput(); // Configure terminal
    
    // Print usage info
    std::cout << "=========================" << std::endl;
    std::cout << "PX4 FSM Command Interface" << std::endl;
    std::cout << "=========================" << std::endl;
    std::cout << "Commands:" << std::endl;
    std::cout << "  l/L - Trigger landing" << std::endl;
    std::cout << "  q/Q - Quit" << std::endl;
    std::cout << "  e/E - Toggle editable mode" << std::endl;
    std::cout << "  t/T - Trigger arming command" << std::endl;
    std::cout << "  r/R - Return to base and land(developing)" << std::endl;
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

                case 'e':
                case 'E':
                    // editable command
                    if (!editable_mode_msg.data) {
                        editable_mode_msg.data = true;
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: e - Editable mode ON\033[0m" << std::endl;
                    } else {
                        editable_mode_msg.data = false;
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: e - Editable mode OFF\033[0m" << std::endl;
                    }
                    editable_mode_pub.publish(editable_mode_msg);
                    break;

                case 'r':
                case 'R':
                    // return to takeoff position then land
                    std::cout << "\033[1;32m[PX4 FSM USER INPUT]: r - Returning to base...\033[0m" << std::endl;
                    rtb_msg.data = true;
                    rtb_pub.publish(rtb_msg);
                    break;

                case 't':
                case 'T':
                    std::cout << "\033[1;32m[PX4 FSM USER INPUT]: t - Triggering arming command...\033[0m" << std::endl;
                    arm_msg.data = true;
                    arm_pub.publish(arm_msg);
                    break;
                    
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