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
 * Created by Zhaohong Liu on 24-12-4.
*/

#include "K_GPEP.h"

void K_GPEP::set2DESDFMap(const Eigen::Ref<Matrix> &sdf_map, double map_x, double map_y) {
    map_size_x_ = map_x;
    map_size_y_ = map_y;

    map_voxel_num_(0) = static_cast<int>(std::round(map_x / resolution_));
    map_voxel_num_(1) = static_cast<int>(std::round(map_y / resolution_));

    global_sdf_buffer_.resize(map_voxel_num_(0) * map_voxel_num_(1), camera_range_max_);

    for (int row = 0; row < map_voxel_num_(0); row++) {
        for (int col = 0; col < map_voxel_num_(1); col++) {
            auto pos2d = voxel2Pos(row, col);
            int index = pos2BufferIndex(pos2d);
            global_sdf_buffer_[index] = sdf_map(row, col);
        }
    }

    esdf_updated_ = true;
}

Eigen::Vector2i K_GPEP::pos2Voxel(const Eigen::Vector2f &pos2d) const {
    Eigen::Vector2i voxel;
    voxel(0) = static_cast<int>(pos2d.x() / resolution_);
    voxel(1) = static_cast<int>(pos2d.y() / resolution_);
    clampVoxel(voxel);
    return voxel;
}

void K_GPEP::clampVoxel(Eigen::Vector2i &voxel) const {
    auto clamp = [](int value, int min_val, int max_val) {
        return std::max(min_val, std::min(max_val, value));
    };
    voxel(0) = clamp(voxel(0), 0, map_voxel_num_(0) - 1);
    voxel(1) = clamp(voxel(1), 0, map_voxel_num_(1) - 1);
}

double K_GPEP::getDistance(const Eigen::Vector3d &pos_3d) const {
    if (!esdf_updated_) {
        return camera_range_max_;
    }
    Eigen::Vector2f pos_2d_f = Eigen::Vector2f(static_cast<float>(pos_3d.x()), static_cast<float>(pos_3d.y()));
    return getDistanceSDF(pos_2d_f);
}

bool K_GPEP::isInMap(const Eigen::Vector3d &pos3d) const {
    Eigen::Vector2f pos2d_f = Eigen::Vector2f(static_cast<float>(pos3d.x()), static_cast<float>(pos3d.y()));
    return isInMap(pos2d_f);
}

Eigen::Vector2f
K_GPEP::singlePseudoRaycast(const Eigen::Vector2f &start, const Eigen::Vector2f &end,
                            bool obs_free_mode) const {
    auto s2e_unit = (end - start).normalized();
    float dist_e2s = (end - start).norm();
    float dist_travel = 0.0;
    Eigen::Vector2f temp_pos = start;
    while (dist_travel < dist_e2s) {
        auto jump_dist = static_cast<float>(getDistanceSDF(temp_pos) - coll_thresh_sq_);

        if (jump_dist >= resolution_) {
            temp_pos += jump_dist * s2e_unit;
            dist_travel += jump_dist;
            if (!isInMap(temp_pos)) {
                return temp_pos - jump_dist * s2e_unit;
            }
        } else {
            temp_pos += resolution_ * s2e_unit;
            dist_travel += resolution_;
            if (!isInMap(temp_pos)) {
                return temp_pos - resolution_ * s2e_unit;
            }
        }


        // available in grid map
//        temp_pos += resolution_ * s2e_unit;
//        dist_travel += resolution_;
//        std::cout << temp_pos.transpose() << ", " << getDistanceSDF(temp_pos) << " ;";
//        if (!isInMap(temp_pos)) {
//            std::cout << ", not in map" << std::endl;
//            return temp_pos - resolution_ * s2e_unit;
//        }

        if (!obs_free_mode && isInflateOccupied(temp_pos)) {
            return temp_pos;
        }
    }

    return end;
}

Eigen::Vector2d
K_GPEP::pseudoRaycast(int direction, const Eigen::Vector2f &start, float theta_init,
                      float d_theta, float radius, const std::string &mode) const {
    Eigen::Vector2f last_occupied_pos = start;  // used for FIND_FREE mode
    float dist2last_occ = 0.f;
    float dist2cur_occ = 0.f;

    Eigen::Vector2f end = start + radius * Eigen::Vector2f(std::cos(theta_init), std::sin(theta_init));
    Eigen::Vector2f temp_pos = singlePseudoRaycast(start, end, false);
    bool temp_pos_occupied = isInflateOccupied(temp_pos);
    if (temp_pos_occupied) {
        last_occupied_pos = temp_pos;
        dist2last_occ = (temp_pos - start).norm();
        if (mode == "FIND_OBSTACLE") {
            // although this should not happen, FIND_OBSTACLE should be used when this temp_pos is free
            return temp_pos.cast<double>();
        }
    }

    float theta_bias = 0.0f; // Use float for precision
    while (theta_bias < sector_range_) {
        theta_bias += d_theta;

        const float theta = theta_init + static_cast<float>(direction) * theta_bias;
        end = start + radius * Eigen::Vector2f(std::cos(theta), std::sin(theta));

        temp_pos = singlePseudoRaycast(start, end, false);
        temp_pos_occupied = isInflateOccupied(temp_pos);

        if (mode == "FIND_OBSTACLE" && temp_pos_occupied) {
            return temp_pos.cast<double>();
        }

        if (mode == "FIND_FREE") {
            if (temp_pos_occupied) {
                dist2cur_occ = (temp_pos - start).norm();

                /*
                 * Last search, the occupied element is last_occupied_pos.
                 * This time, the occupied element is temp_pos.
                 * If the obstacle is continuous,
                 * the distance between them should be less than a specified threshold.
                 * When dist2cur_occ is much larger than dist2last_occ
                 * (there should be an experience gap),
                 * the obstacle might be new,
                 * and there actually is a gap between them to allow the drone to pass.
                 */

                if (dist2cur_occ - dist2last_occ > experience_gap_dist_) {
                    return last_occupied_pos.cast<double>();
                }

                last_occupied_pos = temp_pos;
                dist2last_occ = dist2cur_occ;
            } else if ((temp_pos - end).norm() < 1e-3) {
                return last_occupied_pos.cast<double>();
            }
        }
    }

    // search done, no free space found
    if (mode == "FIND_FREE") {
        return last_occupied_pos.cast<double>();
    }

    return temp_pos.cast<double>();
}

Eigen::Vector4d
K_GPEP::guidedPseudoRaycast(const Eigen::Vector3d &pos, const Eigen::Vector3d &ctrl_p, float preset_radius) const {
    if (!isInMap(pos)) {
        return Eigen::Vector4d::Zero();
    }

    Eigen::Vector2f pos2d = pos.head(2).cast<float>();
    Eigen::Vector2f ctrl_p2d = ctrl_p.head(2).cast<float>();

    float radius;
    if (preset_radius > 0) {
        radius = preset_radius;
    } else {
        radius = (pos2d - ctrl_p2d).norm() + collision_threshold_;
    }

    const float d_theta = std::atan2(resolution_ / 2, radius);
    const float theta_init = std::atan2(ctrl_p2d.y() - pos2d.y(), ctrl_p2d.x() - pos2d.x());
    Eigen::Vector2f raycast_end = pos2d +
            radius * Eigen::Vector2f(std::cos(theta_init), std::sin(theta_init));
    raycast_end = singlePseudoRaycast(pos2d, raycast_end, false);

    Eigen::Vector2d edge_pos_p, edge_pos_n;
    if (isInflateOccupied(raycast_end)) {
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

Corner2d K_GPEP::getGridCorner(const Eigen::Vector2d &pos_2d) const {
    // use std::floor, to get the bottom left corner
    // static_cast<int> won't work for negative values
    auto x = static_cast<float>(pos_2d.x());
    auto y = static_cast<float>(pos_2d.y());
    const float x_bl = std::floor(x / resolution_) * resolution_;
    const float y_bl = std::floor(y / resolution_) * resolution_;

    Corner2d corners = {
            Eigen::Vector2f(x_bl, y_bl),
            Eigen::Vector2f(x_bl + resolution_, y_bl),
            Eigen::Vector2f(x_bl + resolution_, y_bl + resolution_),
            Eigen::Vector2f(x_bl, y_bl + resolution_)
    };

    return corners;
}

Eigen::Vector2d K_GPEP::getPeekCorner(const Eigen::Vector2f &start, const Eigen::Vector2f &end,
                                      const Corner2d &corner, int type) {
    const Eigen::Vector2f start_to_end = end - start;
    float extreme_angle = (type == 0) ? M_PI : 0.0f;  // Use float for precision
    size_t extreme_index = 0;

    for (size_t i = 0; i < corner.size(); ++i) {
        Eigen::Vector2f vec = corner[i] - start;
        float angle = std::acos(std::clamp(start_to_end.dot(vec) /
                                           (start_to_end.norm() * vec.norm()), -1.0f, 1.0f));

        if ((type == 0 && angle < extreme_angle) || (type == 1 && angle > extreme_angle)) {
            extreme_angle = angle;
            extreme_index = i;
        }
    }

    return (corner[extreme_index] - start).cast<double>();
}

Eigen::VectorXd
K_GPEP::getKinematicPseudoRaycast(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                                  const Eigen::Vector3d &ctrl_p) const {
    if (!esdf_updated_) {
        Eigen::VectorXd result(9);
        result.fill(0.0);
        result(8) = camera_range_max_;
        return result;
    }

    double dist_p = (pos - ctrl_p).norm();
    Eigen::Vector3d vel_ctrl = vel.normalized() * dist_p + pos;
    auto edge_vel_cast = guidedPseudoRaycast(pos, vel_ctrl);
    auto edge_ctrl_cast = guidedPseudoRaycast(pos, ctrl_p, experience_radius_);

    Eigen::Vector3d vel_farthest = (ctrl_p - pos).normalized() * camera_range_max_ + pos;
    auto pos2d_f = pos.head(2).cast<float>();
    auto vel_farthest2d_f = vel_farthest.head(2).cast<float>();
    auto vel_farthest_2d = singlePseudoRaycast(pos2d_f, vel_farthest2d_f, false);
    double dist_vel_farthest = (pos2d_f - vel_farthest_2d).norm();

    Eigen::VectorXd result(9);
    result << edge_vel_cast, edge_ctrl_cast, dist_vel_farthest;

    return result;
}

Eigen::VectorXd
K_GPEP::getDoublePseudoRaycast(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel,
                               const Eigen::Vector3d &ctrl_p1, const Eigen::Vector3d &ctrl_p2) const {
    if (!esdf_updated_) {
        Eigen::VectorXd result(11);
        result.fill(0.0);
        result(10) = camera_range_max_;
        return result;
    }
    auto pos2d_f = pos.head(2).cast<float>();

    // get ray cast for ctrl point 1
    auto edge_ctrl_cast1 = guidedPseudoRaycast(pos, ctrl_p1);
    auto pos2ctrl1_farthest = singlePseudoRaycast(pos2d_f, ctrl_p1.head(2).cast<float>(), false);
    double dist_pos2ctrl1 = (pos2d_f - pos2ctrl1_farthest).norm();

    // if ctrl point 1 and 2 is close enough, use the same ray cast
    Eigen::Vector4d edge_ctrl_cast2;
    double dist_pos2ctrl2 = 0.0;
    if ((ctrl_p1 - ctrl_p2).norm() < 1e-2) {
        edge_ctrl_cast2 = edge_ctrl_cast1;
        dist_pos2ctrl2 = dist_pos2ctrl1;
    } else {
        edge_ctrl_cast2 = guidedPseudoRaycast(pos, ctrl_p2);
        auto pos2ctrl2_farthest = singlePseudoRaycast(pos2d_f, ctrl_p2.head(2).cast<float>(), false);
        dist_pos2ctrl2 = (pos2d_f - pos2ctrl2_farthest).norm();
    }

    auto dist_vel_farthest = getVelDistance(pos, vel);

    Eigen::VectorXd result(11);
    result << edge_ctrl_cast1, dist_pos2ctrl1, edge_ctrl_cast2, dist_pos2ctrl2, dist_vel_farthest;
    return result;
}

double K_GPEP::getVelDistance(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel) const {
    auto vel_ctrl = pos + vel.normalized() * camera_range_max_;
    auto pos2d_f = pos.head(2).cast<float>();
    auto vel_farthest2d_f = vel_ctrl.head(2).cast<float>();
    auto vel_farthest_2d = singlePseudoRaycast(pos2d_f, vel_farthest2d_f, false);
    return (pos2d_f - vel_farthest_2d).norm();
}

Eigen::VectorXd K_GPEP::getESDFPerception(const Eigen::Vector3d &pos) const {
    auto pos2d_f = pos.head(2).cast<float>();

    double sdf_values[11];
    if (!esdf_updated_) {
        Eigen::VectorXd result(11);
        result.fill(camera_range_max_);
        result(9) = height_max_ - pos.z();
        result(10) = pos.z() - height_min_;
        return result;
    }

    sdf_values[0] = getDistanceSDF(pos2d_f + Eigen::Vector2f(-resolution_, resolution_));
    sdf_values[1] = getDistanceSDF(pos2d_f + Eigen::Vector2f(0, resolution_));
    sdf_values[2] = getDistanceSDF(pos2d_f + Eigen::Vector2f(resolution_, resolution_));
    sdf_values[3] = getDistanceSDF(pos2d_f + Eigen::Vector2f(-resolution_, 0));
    sdf_values[4] = getDistanceSDF(pos2d_f);
    sdf_values[5] = getDistanceSDF(pos2d_f + Eigen::Vector2f(resolution_, 0));
    sdf_values[6] = getDistanceSDF(pos2d_f + Eigen::Vector2f(-resolution_, -resolution_));
    sdf_values[7] = getDistanceSDF(pos2d_f + Eigen::Vector2f(0, -resolution_));
    sdf_values[8] = getDistanceSDF(pos2d_f + Eigen::Vector2f(resolution_, -resolution_));
    sdf_values[9] = height_max_ - pos.z();
    sdf_values[10] = pos.z() - height_min_;

    Eigen::VectorXd result(11);
    result << sdf_values[0], sdf_values[1], sdf_values[2], sdf_values[3], sdf_values[4],
            sdf_values[5], sdf_values[6], sdf_values[7], sdf_values[8], sdf_values[9], sdf_values[10];

    return result;
}

Eigen::Vector2f K_GPEP::voxel2Pos(int row, int col) const {
    Eigen::Vector2f pos2d;
    float x = static_cast<float>(col) * resolution_;
    float y = static_cast<float>(map_size_y_) - static_cast<float>(row) * resolution_;

    pos2d.x() = x + resolution_ / 2;
    pos2d.y() = y - resolution_ / 2;

    return pos2d;
}

void K_GPEP::clearGlobalESDFBuffer() {
    global_sdf_buffer_.clear();
//    global_sdf_buffer_.shrink_to_fit();
    esdf_updated_ = false;
}
