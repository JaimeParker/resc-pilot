//
// Created by Zhaohong Liu on 24-9-21.
//

#include "px4_utils_land/Convertor.h"

void Convertor::q2EulerAngle(const Eigen::Quaterniond &q, double &roll, double &pitch, double &yaw) {
    double sr_cp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    double cr_cp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sr_cp, cr_cp);

    double sin_p = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sin_p) >= 1)
        pitch = copysign(M_PI / 2, sin_p);  // pi/2
    else
        pitch = asin(sin_p);

    double sy_cp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cy_cp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(sy_cp, cy_cp);
}

void Convertor::q2EulerAngle(const Eigen::Quaterniond &q, Eigen::Vector3d &euler) {
    double roll, pitch, yaw;
    q2EulerAngle(q, roll, pitch, yaw);
    euler << roll, pitch, yaw;
}

void Convertor::euler2Quaternion(Eigen::Quaterniond &q, const Eigen::Vector3d &euler) {
    q = Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
}

Eigen::Matrix3d Convertor::getRotB2A(const Eigen::Vector3d &att) {
    double phi = att[0];
    double theta = att[1];

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1, tan(theta) * sin(phi), tan(theta) * cos(phi),
            0, cos(phi), -sin(phi),
            0, sin(phi) / (cos(theta) + 1e-8), cos(phi) / (cos(theta) + 1e-8);

    return rotation_matrix;
}

Eigen::Matrix3d Convertor::getRotBody2World(const Eigen::Vector3d &att) {
    double s_phi = sin(att[0]);
    double c_phi = cos(att[0]);
    double s_theta = sin(att[1]);
    double c_theta = cos(att[1]);
    double s_psi = sin(att[2]);
    double c_psi = cos(att[2]);

    Eigen::Matrix3d rot_mat;
    rot_mat << c_theta * c_psi, s_theta * s_phi * c_psi - s_psi * c_phi, s_theta * c_phi * c_psi + s_psi * s_phi,
               c_theta * s_psi, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi,
               -s_theta, s_phi * c_theta, c_phi * c_theta;

    return rot_mat;
}

Eigen::Vector3d Convertor::geoMsgsPose2Euler(const geometry_msgs::PoseStamped &pose) {
    Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
                         pose.pose.orientation.y, pose.pose.orientation.z);
    double roll, pitch, yaw;
    q2EulerAngle(q, roll, pitch, yaw);
    return {roll, pitch, yaw};
}

Eigen::Vector3d Convertor::getRate(const Eigen::Vector3d &att_dot, const Eigen::Vector3d &att) {
    auto mat_body2att = getRotB2A(att);
    return mat_body2att.inverse() * att_dot;
}

float Convertor::getThrust(const Eigen::Vector3d &att, const Eigen::Vector3d &accel, const double mass) {
    double g = 9.7946;  // Shanghai
    auto all_force = mass * accel;
    Eigen::Vector3d gravity = {0, 0, -mass * g};
    auto rot_body2world = getRotBody2World(att);
    Eigen::Vector3d thrust_body = rot_body2world.transpose() * (all_force + gravity);
    return static_cast<float>(thrust_body[2]);
}

Eigen::Quaterniond Convertor::euler2Quaternion(const Eigen::Vector3d &euler) {
    return Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());
}
