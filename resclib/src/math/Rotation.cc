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
 * Created by Zhaohong Liu on 24-9-10.
*/

#include "Rotation.h"

Eigen::Matrix3d Rotation::rotB2A(const Eigen::Vector3d &att) {
    double phi = att[0];
    double theta = att[1];

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1, tan(theta) * sin(phi), tan(theta) * cos(phi),
                       0, cos(phi), -sin(phi),
                       0, sin(phi) / (cos(theta) + 1e-8), cos(phi) / (cos(theta) + 1e-8);

    return rotation_matrix;
}

Eigen::Matrix3d Rotation::rotB2ody2World(const Eigen::Vector3d &att) {
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