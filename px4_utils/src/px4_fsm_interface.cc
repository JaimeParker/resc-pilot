//
// Created by Zhaohong Liu on 25-5-21.
//

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/Point.h>

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

    std::string height_change_topic;
    if (!nh.getParam("height_change_topic", height_change_topic)) {
        height_change_topic = "/height_change";
        ROS_WARN("[Land Command]: Using default height change topic: %s", height_change_topic.c_str());
    }

    std::string hold_topic;
    if (!nh.getParam("hold_topic", hold_topic)) {
        hold_topic = "/trigger_hold";
        ROS_WARN("[Land Command]: Using default hold topic: %s", hold_topic.c_str());
    }

    std::string pos_change_topic;
    if (!nh.getParam("position_change_topic", pos_change_topic)) {
        pos_change_topic = "/position_change";
        ROS_WARN("[Land Command]: Using default position change topic: %s", pos_change_topic.c_str());
    }

    std::string yaw_change_topic;
    if (!nh.getParam("yaw_change_topic", yaw_change_topic)) {
        yaw_change_topic = "/yaw_change";
        ROS_WARN("[Land Command]: Using default yaw change topic: %s", yaw_change_topic.c_str());
    }
    
    ros::Publisher land_pub = nh.advertise<std_msgs::Bool>(land_topic, 10);
    ros::Publisher editable_mode_pub = nh.advertise<std_msgs::Bool>(editable_mode_topic, 10);
    ros::Publisher rtb_pub = nh.advertise<std_msgs::Bool>(rtb_topic, 10);
    ros::Publisher arm_pub = nh.advertise<std_msgs::Bool>(arming_topic, 10);
    ros::Publisher height_change_pub = nh.advertise<std_msgs::Float32>(height_change_topic, 10);
    ros::Publisher hold_pub = nh.advertise<std_msgs::Bool>(hold_topic, 10);
    ros::Publisher pos_change_pub = nh.advertise<geometry_msgs::Point>(pos_change_topic, 10);
    ros::Publisher yaw_change_pub = nh.advertise<std_msgs::Float32>(yaw_change_topic, 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    std_msgs::Bool land_msg;
    land_msg.data = true;

    std_msgs::Bool editable_mode_msg;
    editable_mode_msg.data = false;

    std_msgs::Bool rtb_msg;
    rtb_msg.data = false;

    std_msgs::Bool arm_msg;
    arm_msg.data = false;

    std_msgs::Bool hold_msg;
    hold_msg.data = false;
    
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
    std::cout << "  h/H - Switch to HOLD mode" << std::endl;
    std::cout << "  w/W - Move forward by 0.1m" << std::endl;
    std::cout << "  a/A - Move left by 0.1m" << std::endl;
    std::cout << "  s/S - Move backward by 0.1m" << std::endl;
    std::cout << "  d/D - Move right by 0.1m" << std::endl;
    std::cout << "  i/I - Increase yaw by 10 degrees" << std::endl;
    std::cout << "  p/P - Decrease yaw by 10 degrees" << std::endl;
    std::cout << "  (Fn)+PgUp - Increase height by 0.1m" << std::endl;
    std::cout << "  (Fn)+PgDn - Decrease height by 0.1m" << std::endl;
    std::cout << "=========================" << std::endl;
    
    ros::Rate rate(10);
    
    while (ros::ok()) {
        char buf[5] = {0};
        int bytes_read = read(STDIN_FILENO, buf, sizeof(buf) - 1);
        geometry_msgs::Point pos_change_msg;
        std_msgs::Float32 yaw_change_msg;

        if (bytes_read > 0) {
            std::cout << "\r\033[K"; // Clear the current line
            if (bytes_read == 1) {
                // Handle single character input
                switch (buf[0]) {
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

                    case 'h':
                    case 'H':
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: h - Switching to HOLD mode...\033[0m" << std::endl;
                        hold_msg.data = true;
                        hold_pub.publish(hold_msg);
                        break;

                    case 'w':
                    case 'W':
                        pos_change_msg.x = 0.1f;
                        pos_change_pub.publish(pos_change_msg);
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: w - Move forward by 0.1m\033[0m" << std::endl;
                        break;

                    case 'a':
                    case 'A':
                        pos_change_msg.y = 0.1f;
                        pos_change_pub.publish(pos_change_msg);
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: a - Move left by 0.1m\033[0m" << std::endl;
                        break;

                    case 's':
                    case 'S':
                        pos_change_msg.x = -0.1f;
                        pos_change_pub.publish(pos_change_msg);
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: s - Move backward by 0.1m\033[0m" << std::endl;
                        break;

                    case 'd':
                    case 'D':
                        pos_change_msg.y = -0.1f;
                        pos_change_pub.publish(pos_change_msg);
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: d - Move right by 0.1m\033[0m" << std::endl;
                        break;

                    case 'i':
                    case 'I':
                        yaw_change_msg.data = 10.0f;
                        yaw_change_pub.publish(yaw_change_msg);
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: i - Increase yaw by 10 degrees\033[0m" << std::endl;
                        break;

                    case 'p':
                    case 'P':
                        yaw_change_msg.data = -10.0f;
                        yaw_change_pub.publish(yaw_change_msg);
                        std::cout << "\033[1;32m[PX4 FSM USER INPUT]: p - Decrease yaw by 10 degrees\033[0m" << std::endl;
                        break;

                    default:
                        std::cout << "\033[1;37m[PX4 FSM USER INPUT]: '" << buf[0]
                                  << "' - Unknown command\033[0m" << std::endl;
                        break;
                }
            } else if (bytes_read == 4 && buf[0] == '\x1b' && buf[1] == '[' && buf[3] == '~') {
                // Handle multi-byte escape sequences
                std_msgs::Float32 height_msg;
                if (buf[2] == '5') { // Page Up
                    height_msg.data = 0.1f;
                    height_change_pub.publish(height_msg);
                    std::cout << "\033[1;32m[PX4 FSM USER INPUT]: PgUp - Increase height by 0.1m\033[0m" << std::endl;
                } else if (buf[2] == '6') { // Page Down
                    height_msg.data = -0.1f;
                    height_change_pub.publish(height_msg);
                    std::cout << "\033[1;32m[PX4 FSM USER INPUT]: PgDown - Decrease height by 0.1m\033[0m" << std::endl;
                } else {
                    std::cout << "[DEBUG]: Unknown escape sequence: " 
                            << std::hex << (int)buf[0] << " " 
                            << (int)buf[1] << " " 
                            << (int)buf[2] << " " 
                            << (int)buf[3] << std::endl;
                }
            }

        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    restoreTerminal(); // Restore terminal settings when exiting
    return 0;
}