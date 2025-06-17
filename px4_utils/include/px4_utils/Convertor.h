//
// Created by Zhaohong Liu on 24-9-21.
//

#ifndef PX4_UTILS_CONVERTOR_H
#define PX4_UTILS_CONVERTOR_H

#include <Eigen/Eigen>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>

class Convertor {
public:
    static void q2EulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

    static void q2EulerAngle(const Eigen::Quaterniond& q, Eigen::Vector3d& euler);

    static void euler2Quaternion(Eigen::Quaterniond& q, const Eigen::Vector3d& euler);

    static Eigen::Quaterniond euler2Quaternion(const Eigen::Vector3d& euler);

    static Eigen::Matrix3d getRotB2A(const Eigen::Vector3d &att);

    static Eigen::Matrix3d getRotBody2World(const Eigen::Vector3d &att);

    static Eigen::Vector3d geoMsgsPose2Euler(const geometry_msgs::PoseStamped& pose);

    static Eigen::Vector3d getRate(const Eigen::Vector3d& att_dot, const Eigen::Vector3d& att);

    static float getThrust(const Eigen::Vector3d &att, const Eigen::Vector3d &accel, double mass);
};


#endif //PX4_UTILS_CONVERTOR_H
