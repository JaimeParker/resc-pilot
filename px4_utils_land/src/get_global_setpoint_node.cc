/**
 * @file get_global_setpoint_node.cc
 * @brief Node to capture GPS coordinates and calculate relative ENU position
 * 
 * Functionality:
 * 1. Capture GPS coordinates (lat, lon, alt) when drone powers on
 * 2. Capture GPS coordinates when user presses a key (ENTER)
 * 3. Calculate relative position vector in ENU frame (meters)
 * 4. Save both coordinates and relative vector to XML file
 */

#include <ros/ros.h>
#include <ros/package.h>  
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

class GlobalSetpointRecorder {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    
    // GPS data storage
    sensor_msgs::NavSatFix initial_gps_;
    sensor_msgs::NavSatFix target_gps_;
    bool initial_captured_ = false;
    bool target_captured_ = false;
    bool gps_received_ = false;
    
    // Output file path
    std::string output_file_;
    
    // Earth parameters
    const double EARTH_RADIUS = 6378137.0; // WGS84 equatorial radius in meters
    
public:
    GlobalSetpointRecorder() : nh_("~") {
        // Get output file path from parameter, default to resc-pilot/launch folder
        std::string package_path = ros::package::getPath("px4_utils_land");
        std::string relative_path = "launch/global_setpoints.yaml";
        std::string default_path = package_path + "/" + relative_path;
        nh_.param<std::string>("output_file", output_file_, default_path);

        // Read target GPS coordinates from parameters
        double target_lat, target_lon, target_alt;
        if (nh_.getParam("target_latitude", target_lat) && 
            nh_.getParam("target_longitude", target_lon) && 
            nh_.getParam("target_altitude", target_alt)) {
            
            target_gps_.latitude = target_lat;
            target_gps_.longitude = target_lon;
            target_gps_.altitude = target_alt;
            target_gps_.status.status = 0; // Set valid status
            target_captured_ = true;
        }

        // Subscribe to GPS topic
        gps_sub_ = nh_.subscribe("/mavros/global_position/global", 10, 
                                 &GlobalSetpointRecorder::gpsCallback, this);
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!gps_received_) {
            gps_received_ = true;
        }
        
        // Capture initial GPS coordinates on first valid GPS message
        if (!initial_captured_ && msg->status.status >= 0) {
            initial_gps_ = *msg;
            initial_captured_ = true;
        }
        
        // Update current GPS for manual capture
        if (initial_captured_ && !target_captured_) {
            target_gps_ = *msg;
        }
    }
    
    /**
     * @brief Convert GPS coordinates to ENU relative position
     * @param ref Reference GPS coordinate (initial position)
     * @param target Target GPS coordinate
     * @return Vector3 with ENU coordinates (East, North, Up) in meters
     */
    geometry_msgs::Vector3 gpsToENU(const sensor_msgs::NavSatFix& ref, 
                                     const sensor_msgs::NavSatFix& target) {
        geometry_msgs::Vector3 enu;
        
        // Convert degrees to radians
        double lat1 = ref.latitude * M_PI / 180.0;
        double lon1 = ref.longitude * M_PI / 180.0;
        double lat2 = target.latitude * M_PI / 180.0;
        double lon2 = target.longitude * M_PI / 180.0;
        
        // Calculate differences
        double dlat = lat2 - lat1;
        double dlon = lon2 - lon1;
        
        // Calculate ENU coordinates
        // East: delta_longitude * R * cos(latitude)
        enu.x = dlon * EARTH_RADIUS * std::cos((lat1 + lat2) / 2.0);
        
        // North: delta_latitude * R
        enu.y = dlat * EARTH_RADIUS;
        
        // Up: delta_altitude
        enu.z = 5.0; // Adding 5.0 to account for altitude offset

        return enu;
    }
    
    /**
     * @brief Calculate Euclidean distance between two GPS points
     */
    double calculateDistance(const sensor_msgs::NavSatFix& ref, 
                            const sensor_msgs::NavSatFix& target) {
        geometry_msgs::Vector3 enu = gpsToENU(ref, target);
        return std::sqrt(enu.x * enu.x + enu.y * enu.y + enu.z * enu.z);
    }
    
    
    /**
     * @brief Main loop to handle keyboard input
     */
    void run() {
        // Wait for initial GPS capture
        ros::Rate rate(10);
        while (ros::ok() && !initial_captured_) {
            ros::spinOnce();
            rate.sleep();
        }
        
        if (!ros::ok()) {
            return;
        }
        
        // If target GPS is already loaded from parameters, process immediately
        if (target_captured_) {

            // Calculate and display relative position
            geometry_msgs::Vector3 enu = gpsToENU(initial_gps_, target_gps_);
            double dist = calculateDistance(initial_gps_, target_gps_);

            std::string input;
            std::getline(std::cin, input);

            if (input == "c" || input == "C") {
                ROS_WARN("User cancelled. Node shutting down without saving.");
                return;
            }

            // Export to ROS parameters under /global_gps namespace so launch-time yaml is not needed
            ros::NodeHandle gnh;
            gnh.setParam("/global_gps/initial/latitude", initial_gps_.latitude);
            gnh.setParam("/global_gps/initial/longitude", initial_gps_.longitude);
            gnh.setParam("/global_gps/initial/altitude", initial_gps_.altitude);

            gnh.setParam("/global_gps/target/latitude", target_gps_.latitude);
            gnh.setParam("/global_gps/target/longitude", target_gps_.longitude);
            gnh.setParam("/global_gps/target/altitude", target_gps_.altitude);

            geometry_msgs::Vector3 enu_vec = gpsToENU(initial_gps_, target_gps_);
            gnh.setParam("/global_gps/enu_relative/east", enu_vec.x);
            gnh.setParam("/global_gps/enu_relative/north", enu_vec.y);
            gnh.setParam("/global_gps/enu_relative/up", enu_vec.z);

            double distance = calculateDistance(initial_gps_, target_gps_);
            gnh.setParam("/global_gps/enu_relative/distance", distance);

            // Signal readiness to the rest of the system
            gnh.setParam("/global_gps/ready", true);

            return;
        }


        // Wait for ENTER from user to capture current GPS as target
        std::string line;
        std::getline(std::cin, line);
        if (!ros::ok()) return;

        // mark target captured and process
        target_captured_ = true;

        geometry_msgs::Vector3 enu = gpsToENU(initial_gps_, target_gps_);
        double dist = calculateDistance(initial_gps_, target_gps_);

        ROS_INFO("\nRelative Position (ENU):");
        ROS_INFO("  East:  %.3f m", enu.x);
        ROS_INFO("  North: %.3f m", enu.y);
        ROS_INFO("  Up:    %.3f m", enu.z);
        ROS_INFO("  Distance: %.3f m", dist);

        std::cout << "Confirm these values? Press ENTER to confirm, or 'c' then ENTER to cancel: ";
        std::string input2;
        std::getline(std::cin, input2);
        if (input2 == "c" || input2 == "C") {
            ROS_WARN("User cancelled. Node shutting down without saving.");
            return;
        }

        // Save and export params
        ros::NodeHandle gnh;
        gnh.setParam("/global_gps/initial/latitude", initial_gps_.latitude);
        gnh.setParam("/global_gps/initial/longitude", initial_gps_.longitude);
        gnh.setParam("/global_gps/initial/altitude", initial_gps_.altitude);

        gnh.setParam("/global_gps/target/latitude", target_gps_.latitude);
        gnh.setParam("/global_gps/target/longitude", target_gps_.longitude);
        gnh.setParam("/global_gps/target/altitude", target_gps_.altitude);

        gnh.setParam("/global_gps/enu_relative/east", enu.x);
        gnh.setParam("/global_gps/enu_relative/north", enu.y);
        gnh.setParam("/global_gps/enu_relative/up", enu.z);
        gnh.setParam("/global_gps/enu_relative/distance", dist);
        gnh.setParam("/global_gps/ready", true);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "get_global_setpoint_node");
    
    GlobalSetpointRecorder recorder;
    recorder.run();
    
    return 0;
}
