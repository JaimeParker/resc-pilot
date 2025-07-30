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
 * Created by Zhaohong Liu on 24-9-21.
*/

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
