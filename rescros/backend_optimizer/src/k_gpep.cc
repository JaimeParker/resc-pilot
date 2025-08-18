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
 * Created by Zhaohong Liu on 24-11-30.
*/

#include "backend_optimizer/k_gpep.h"

void K_GPEP::setMapBridge(const MapBridge::Ptr &ptr) {
    map_bridge_ptr_ = ptr;

    collision_threshold_ = ptr->getCollisionThreshold();
    res_ = ptr->getResolution();
}

std::vector<double> K_GPEP::solve(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &ctrl_p) {
    double dist = (pos - ctrl_p).norm();
    Eigen::Vector3d vel_ctrl = vel.normalized() * dist + pos;

    Eigen::Vector4d edge_vel_cast = guidedPseudoRaycast(pos, vel_ctrl);
    Eigen::Vector4d edge_pos_cast = guidedPseudoRaycast(pos, ctrl_p, 2.0);

    Eigen::Vector3d vel_farthest = (ctrl_p - pos).normalized() * 3.0 + pos;
    Eigen::Vector2d vel_farthest_2d = singlePseudoRaycast(pos.head(2), vel_farthest.head(2), false);
    double dist_vel_farthest = (pos.head(2) - vel_farthest_2d).norm();

    double sdf_values[11];
    Eigen::Vector3d pos_tl = pos + Eigen::Vector3d(-res_, res_, 0);
    Eigen::Vector3d pos_t = pos + Eigen::Vector3d(0, res_, 0);
    Eigen::Vector3d pos_tr = pos + Eigen::Vector3d(res_, res_, 0);
    Eigen::Vector3d pos_l = pos + Eigen::Vector3d(-res_, 0, 0);
    Eigen::Vector3d pos_r = pos + Eigen::Vector3d(res_, 0, 0);
    Eigen::Vector3d pos_bl = pos + Eigen::Vector3d(-res_, -res_, 0);
    Eigen::Vector3d pos_b = pos + Eigen::Vector3d(0, -res_, 0);
    Eigen::Vector3d pos_br = pos + Eigen::Vector3d(res_, -res_, 0);

    sdf_values[0] = map_bridge_ptr_->getDistance(pos_tl);
    sdf_values[1] = map_bridge_ptr_->getDistance(pos_t);
    sdf_values[2] = map_bridge_ptr_->getDistance(pos_tr);
    sdf_values[3] = map_bridge_ptr_->getDistance(pos_l);
    sdf_values[4] = map_bridge_ptr_->getDistance(pos);
    sdf_values[5] = map_bridge_ptr_->getDistance(pos_r);
    sdf_values[6] = map_bridge_ptr_->getDistance(pos_bl);
    sdf_values[7] = map_bridge_ptr_->getDistance(pos_b);
    sdf_values[8] = map_bridge_ptr_->getDistance(pos_br);
    sdf_values[9] = h_top_ - pos.z();
    sdf_values[10] = pos.z() - h_bottom_;

    std::vector<double> result(sdf_values, sdf_values + 11);

    result.insert(result.end(), edge_vel_cast.data(), edge_vel_cast.data() + edge_vel_cast.size());
    result.insert(result.end(), edge_pos_cast.data(), edge_pos_cast.data() + edge_pos_cast.size());

    result.push_back(dist_vel_farthest);

    return result;
}

Eigen::Vector4d K_GPEP::guidedPseudoRaycast(const Eigen::Vector3d &pos, const Eigen::Vector3d &ctrl_p,
                                            const float preset_radius) {
    if (!map_bridge_ptr_->isInMap(pos)) {
        return Eigen::Vector4d::Zero();
    }

    Eigen::Vector2d pos2d = pos.head(2);
    Eigen::Vector2d ctrl_p2d = ctrl_p.head(2);

    double radius;
    if (preset_radius > 0) {
        radius = preset_radius;
    } else {
        radius = (pos2d - ctrl_p2d).norm() + collision_threshold_;
    }

    const double d_theta = std::atan2(res_, radius);
    const double theta_init = std::atan2(ctrl_p2d[1] - pos2d[1], ctrl_p2d[0] - pos2d[0]);
    Eigen::Vector2d raycast_end = pos2d +
            radius * Eigen::Vector2d(std::cos(theta_init), std::sin(theta_init));
    raycast_end = singlePseudoRaycast(pos2d, raycast_end, false);
    Eigen::Vector3d raycast_end_3d = Eigen::Vector3d(raycast_end.x(), raycast_end.y(), cruise_height_);

    Eigen::Vector2d edge_pos_p, edge_pos_n;
    if (map_bridge_ptr_->isInflateOccupied(raycast_end_3d)) {
        const auto pos_p = pseudoRaycast(1, pos2d, theta_init, d_theta, radius, "FIND_FREE");
        const auto pos_n = pseudoRaycast(-1, pos2d, theta_init, d_theta, radius, "FIND_FREE");
        const auto corner_p = getGridCorner(pos_p);
        const auto corner_n = getGridCorner(pos_n);
        edge_pos_p = getPeekCorner(pos2d, ctrl_p2d, corner_p, 1);
        edge_pos_n = getPeekCorner(pos2d, ctrl_p2d, corner_n, 1);
    } else {
        const auto pos_p = pseudoRaycast(1, pos2d, theta_init, d_theta, radius, "FIND_OBSTACLE");
        const auto pos_n = pseudoRaycast(-1, pos2d, theta_init, d_theta, radius, "FIND_OBSTACLE");
        const auto corner_p = getGridCorner(pos_p);
        const auto corner_n = getGridCorner(pos_n);
        edge_pos_p = getPeekCorner(pos2d, ctrl_p2d, corner_p, 0);
        edge_pos_n = getPeekCorner(pos2d, ctrl_p2d, corner_n, 0);
    }

    Eigen::Vector4d edge_pos;
    edge_pos << edge_pos_p, edge_pos_n;
    return edge_pos;
}

Eigen::Vector2d K_GPEP::singlePseudoRaycast(const Eigen::Vector2d &start, const Eigen::Vector2d &end,
                                            bool obs_free_mode) {
    Eigen::Vector3d temp_pos_3d(start.x(), start.y(), cruise_height_);
    auto d_pos = (end - start).normalized() * res_;
    // ignore z
    Eigen::Vector3d d_pos_3d = Eigen::Vector3d(d_pos.x(), d_pos.y(), 0);

    double length = (end - start).norm();
    int num_steps = static_cast<int>(std::round(length / res_));

    for (int i = 0; i < num_steps; i++) {
        temp_pos_3d += d_pos_3d;

        if (!map_bridge_ptr_->isInMap(temp_pos_3d)) {
            return (temp_pos_3d - d_pos_3d).head(2);
        }

        if (!obs_free_mode && map_bridge_ptr_->isInflateOccupied(temp_pos_3d)) {
            return temp_pos_3d.head(2);
        }
    }

    return end;
}

Eigen::Vector2d K_GPEP::pseudoRaycast(int direction, const Eigen::Vector2d &start, double theta_init,
                                      double d_theta, double radius, const std::string &mode) {
    Eigen::Vector2d last_occupied_pos = start;
    Eigen::Vector2d temp_pos = start;

    double theta_bias = 0.0;
    while (theta_bias < sector_range_) {
        const double theta = theta_init + direction * theta_bias;
        Eigen::Vector2d end = start + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));

        if (!map_bridge_ptr_->isInMap(Eigen::Vector3d(end.x(), end.y(), cruise_height_))) {
            return Eigen::Vector2d::Zero();
        }

        temp_pos = singlePseudoRaycast(start, end, false);

        if (mode == "FIND_OBSTACLE" &&
            map_bridge_ptr_->isInflateOccupied(Eigen::Vector3d(end.x(), end.y(), cruise_height_))) {
            return temp_pos;
        }

        if (mode == "FIND_FREE") {
            if (map_bridge_ptr_->isInflateOccupied(Eigen::Vector3d(temp_pos.x(), temp_pos.y(), cruise_height_))) {
                last_occupied_pos = temp_pos;
            } else if ((temp_pos - end).norm() < 1e-3) {
                return last_occupied_pos;
            }
        }

        theta_bias += d_theta;
    }

    if (mode == "FIND_FREE") {
        return last_occupied_pos;
    }

    return temp_pos;
}

Corner K_GPEP::getGridCorner(const Eigen::Vector2d &pos_2d) const {
    const double x_bl = static_cast<int>(pos_2d.x() / res_) * res_;
    const double y_bl = static_cast<int>(pos_2d.y() / res_) * res_;

    std::array<Eigen::Vector2d, 4> corners = {
            Eigen::Vector2d(x_bl, y_bl),
            Eigen::Vector2d(x_bl + res_, y_bl),
            Eigen::Vector2d(x_bl + res_, y_bl + res_),
            Eigen::Vector2d(x_bl, y_bl + res_)
    };

    return corners;
}

Eigen::Vector2d K_GPEP::getPeekCorner(const Eigen::Vector2d &start, const Eigen::Vector2d &end,
                                      const Corner &corners, int type) {
    const Eigen::Vector2d start_to_end = end - start;

    double extreme_angle = (type == 0) ? M_PI : 0;  // 0 MIN, 1 MAX
    size_t extreme_index = 0;

    for (size_t i = 0; i < corners.size(); ++i) {
        Eigen::Vector2d vec = corners[i] - start;
        const double angle = std::acos(std::clamp(start_to_end.dot(vec)
                                                  / (start_to_end.norm() * vec.norm()), -1.0, 1.0));

        if ((type == 0 && angle < extreme_angle) || (type == 1 && angle > extreme_angle)) {
            extreme_angle = angle;
            extreme_index = i;
        }
    }

    return corners[extreme_index] - start;
}
