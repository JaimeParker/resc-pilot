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
 * Created by Zhaohong Liu on 24-9-3.
*/

#include "SignedDistanceField.h"

bool SignedDistanceField::isCollide(const double distance) const {
    return distance < collision_distance_;
}

bool SignedDistanceField::isValidIndex(const Index2d &index) const {
    return 1 <= index.first && index.first < map_size_int_ - 1 &&
           1 <= index.second && index.second < map_size_int_ - 1;
}

bool SignedDistanceField::isEdgeVoxel(const Index2d &index) const {
    return index.first == 1 || index.first == map_size_int_ - 2 ||
           index.second == 1 || index.second == map_size_int_ - 2;
}

template<typename EigenVector>
bool SignedDistanceField::isValidPos(const EigenVector &pos) const {
    return 0 <= pos[0] && pos[0] < map_size_ &&
           0 <= pos[1] && pos[1] < map_size_;
}

template<typename EigenVector>
Index2d SignedDistanceField::pos2Index(const EigenVector &pos) const {
    Index2d pos_index;
    const double x = pos[0];
    const double y = map_size_ - pos[1];

    pos_index.first = static_cast<int>(y / resolution_);
    pos_index.second = static_cast<int>(x / resolution_);

//    pos_index.first = static_cast<int>(std::floor(y / resolution_));
//    pos_index.second = static_cast<int>(std::floor(x / resolution_));

    return pos_index;
}

Corner SignedDistanceField::getGridCorner(const Index2d &index) const {
    const double x_center = index.second * resolution_ + resolution_ / 2;
    const double y_center = map_size_ - (index.first * resolution_ + resolution_ / 2);

    const double half_size = resolution_ / 2;

    std::array<Eigen::Vector2d, 4> corners = {
            Eigen::Vector2d(x_center - half_size, y_center + half_size),  // top_left
            Eigen::Vector2d(x_center + half_size, y_center + half_size),  // top_right
            Eigen::Vector2d(x_center - half_size, y_center - half_size),  // bottom_left
            Eigen::Vector2d(x_center + half_size, y_center - half_size)   // bottom_right
    };

    return corners;
}

Eigen::Vector2d SignedDistanceField::getPeekCorner(const Eigen::Vector2d &pos2d,
                                                   const Eigen::Vector2d &ctrl_p2d,
                                                   const Corner &corners,
                                                   const AngleType type) {
    const Eigen::Vector2d pos2end = ctrl_p2d - pos2d;

    double extreme_angle = (type == MIN) ? M_PI : 0;
    size_t extreme_index = 0;

    for (size_t i = 0; i < corners.size(); ++i) {
        Eigen::Vector2d vec = corners[i] - pos2d;
        const double angle = std::acos(std::clamp(pos2end.dot(vec)
                                                  / (pos2end.norm() * vec.norm()), -1.0, 1.0));

        if ((type == MIN && angle < extreme_angle) ||
            (type == MAX && angle > extreme_angle)) {
            extreme_angle = angle;
            extreme_index = i;
        }
    }

    return corners[extreme_index] - pos2d;
}

Eigen::Vector4d SignedDistanceField::rayCastingEdge(const Eigen::Vector3d &pos,
                                                    const Eigen::Vector3d &ctrl_p,
                                                    const Eigen::Ref<Matrix> &sdf_map,
                                                    const float preset_radius) const {
    const Index2d pos_index = pos2Index(pos);
    if (!isValidIndex(pos_index)) {
        return Eigen::Vector4d::Zero();
    }

    Eigen::Vector2d pos2d = pos.head(2);
    Eigen::Vector2d ctrl_p2d = ctrl_p.head(2);

    double radius;
    if (preset_radius > 0) {
        radius = preset_radius;
    } else {
        radius = (pos2d - ctrl_p2d).norm() + collision_distance_;
    }

    const double d_theta = std::atan2(resolution_, radius);
    const double theta_init = std::atan2(ctrl_p2d[1] - pos2d[1], ctrl_p2d[0] - pos2d[0]);

    Eigen::Vector2d radius_ray_end = pos2d +
            radius * Eigen::Vector2d(std::cos(theta_init), std::sin(theta_init));
    Index2d rre_index = pos2Index(Eigen::Vector3d(radius_ray_end[0], radius_ray_end[1], 0));
    rre_index = bresenhamFarthestIndex(pos_index, rre_index, false, sdf_map);

    Eigen::Vector2d edge_pos_p, edge_pos_n;
    if (isCollide(sdf_map(rre_index.first, rre_index.second))) {
        const auto index_p =
            sectorRayCasting(1, pos2d, theta_init, d_theta, radius, "FIND_FREE", sdf_map);
        const auto index_n =
            sectorRayCasting(-1, pos2d, theta_init, d_theta, radius, "FIND_FREE", sdf_map);
        const auto corner_p = getGridCorner(index_p);
        const auto corner_n = getGridCorner(index_n);
        edge_pos_p = getPeekCorner(pos2d, ctrl_p2d, corner_p, MAX);
        edge_pos_n = getPeekCorner(pos2d, ctrl_p2d, corner_n, MAX);
    } else {
        const auto index_p =
            sectorRayCasting(1, pos2d, theta_init, d_theta, radius, "FIND_OBSTACLE", sdf_map);
        const auto index_n =
            sectorRayCasting(-1, pos2d, theta_init, d_theta, radius, "FIND_OBSTACLE", sdf_map);
        const auto corner_p = getGridCorner(index_p);
        const auto corner_n = getGridCorner(index_n);
        edge_pos_p = getPeekCorner(pos2d, ctrl_p2d, corner_p, MIN);
        edge_pos_n = getPeekCorner(pos2d, ctrl_p2d, corner_n, MIN);
    }

    Eigen::Vector4d edge_pos;
    edge_pos << edge_pos_p, edge_pos_n;
    return edge_pos;
}

Index2d SignedDistanceField::sectorRayCasting(const int direction,
                                              const Eigen::Vector2d& pos_2d,
                                              const double theta_init,
                                              const double d_theta,
                                              const double radius,
                                              const std::string& mode,
                                              const Eigen::Ref<Matrix> &sdf_map) const {
    const Index2d pos_index = pos2Index(pos_2d);
    Index2d last_occupied_index = {-1, -1};
    Index2d voxel_index = {-1, -1};

    double theta_bias = 0.0;
    while (theta_bias < sector_range_) {
        const double theta = theta_init + direction * theta_bias;
        Eigen::Vector2d end_pos = pos_2d + radius * Eigen::Vector2d(std::cos(theta), std::sin(theta));

        if (!isValidPos(pos_2d)) {
            return {0, 0};
        }

        auto end_index = pos2Index(end_pos);
        voxel_index = bresenhamFarthestIndex(pos_index, end_index, false, sdf_map);

        if (mode == "FIND_OBSTACLE" && isCollide(sdf_map(voxel_index.first, voxel_index.second))) {
            return voxel_index;
        }

        if (mode == "FIND_FREE") {
            if (isCollide(sdf_map(voxel_index.first, voxel_index.second))) {
                last_occupied_index = voxel_index;
            } else if ((voxel_index == end_index) || isEdgeVoxel(voxel_index)) {
                return last_occupied_index;
            }
        }

        theta_bias += d_theta;
    }

    if (mode == "FIND_FREE") {
        return last_occupied_index;
    }
    return voxel_index;
    //    return (mode == "FIND_FREE") ? last_occupied_index : voxel_index;
}

Index2d SignedDistanceField::bresenhamFarthestIndex(const Index2d &start,
                                                    const Index2d &end,
                                                    const bool obs_free_mode,
                                                    const Eigen::Ref<Matrix> &sdf_map) const {
    // TODO: 将bresenham算法改为连续的形式
    // 将参数改为start pos, end pos，之后可以构建start到end的参数方程，然后循环增加dt，直到end
    // 每次循环中，对于更新出的 start pos + dt (end - start)的index，判断是否是边界或障碍物
    // 最后输出start到end这条线段上，距离start最远的非边界且非障碍物的index
    int row0 = start.first;
    int col0 = start.second;

    // Bresenham preprocessing
    int dx = std::abs(end.first - row0);
    int dy = std::abs(end.second - col0);
    const int sx = (row0 < end.first) ? 1 : -1;
    const int sy = (col0 < end.second) ? 1 : -1;
    int err = dx - dy;

    auto last_index = start;

    while (true) {
        if (!isValidIndex({row0, col0})) {
            return last_index;
        }

        last_index = {row0, col0};

        if (!obs_free_mode && isCollide(sdf_map(row0, col0))) {
            return {row0, col0};
        }

        if (row0 == end.first && col0 == end.second) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            row0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            col0 += sy;
        }
    }

    return end;
}

void SignedDistanceField::resetMapSize(double &map_size) {
    // clarify the map to avoid out of range
    map_size_ = std::round(map_size / resolution_) * resolution_;
    map_size_int_ = static_cast<int>(map_size_ / resolution_);
}

