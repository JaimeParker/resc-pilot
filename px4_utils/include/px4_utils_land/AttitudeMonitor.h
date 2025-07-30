//
// Created by Zhaohong Liu on 24-9-25.
//

#ifndef PX4_UTILS_LAND_ATTITUDEMONITOR_H
#define PX4_UTILS_LAND_ATTITUDEMONITOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>


#include "px4_utils_land/Convertor.h"

enum Status {
    SYSTEM_ID,
    RTB,
    HOLD,
};

class AttitudeMonitor {
public:
    void init(ros::NodeHandle &nh);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void rateCallback(const sensor_msgs::Imu::ConstPtr &msg);
    bool isAggressive();
    bool isOutOfGeoFence();
    void initSystemID();
    void runSystemIdentification();
    bool holdPosition(double duration);
    bool backToOrigin();
    void setPoseMsg(const Eigen::Vector3d & pos, const Eigen::Vector3d & att);
    void setRate(int angle_type);

private:
    /* ros utils */
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    std::string pose_sub_topic_ = "/mavros/local_position/pose";
    ros::Publisher pose_pub_;
    geometry_msgs::PoseStamped pose_pub_msg_;
    std::string pose_pub_topic_ = "mavros/setpoint_position/local";
    std::string pose_pub_frame_id_ = "world";

    ros::Subscriber rate_sub_;
    std::string rate_sub_topic_ = "/mavros/imu/data";
    mavros_msgs::AttitudeTarget rate_pub_msg_;
    ros::Publisher rate_pub_;


    /* drone states */
    Eigen::Vector3d rate_;
    Eigen::Vector3d att_;
    Eigen::Vector3d pos_;
    Status status_ = SYSTEM_ID;

    /* aggressive motion definition */
    const double deg2rad_ = M_PI / 180;
    const double roll_tol_ = 20 * deg2rad_;
    const double pitch_tol_ = 20 * deg2rad_;
    const double geo_fence_x_ = 1.0;
    const double geo_fence_y_ = 1.0;
    const double geo_fence_z_ = 1.75;

    /* system id utils */
    int rate_execution_count_ = 0;
    ros::Time frozen_time_;
    Eigen::Vector3d frozen_pos_;
    Eigen::Vector3d frozen_att_;
    Eigen::Vector3d origin_pos_ = {0.0, 0.0, 1.0};
    Eigen::Vector3d origin_att_ = {0.0, 0.0, 0.0};
    const double delta_rate_ = 15 * deg2rad_;
    int rate_type_ = 0;
    int each_rate_exe_times = 3;
};


#endif //PX4_UTILS_LAND_ATTITUDEMONITOR_H
