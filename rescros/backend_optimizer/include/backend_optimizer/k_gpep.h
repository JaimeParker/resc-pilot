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
 * Kinematic Guided Pseudo-Raycast ESDF Perception
*/

#ifndef BACKEND_OPTIMIZER_K_GPEP_H
#define BACKEND_OPTIMIZER_K_GPEP_H

#include <Eigen/Eigen>

#include <map_utils/MapBridge.h>

using Corner = std::array<Eigen::Vector2d, 4>;

class K_GPEP {
private:
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;

    MapBridge::Ptr map_bridge_ptr_;

    /* params */
    double collision_threshold_ = 0.3;
    double res_ = 0.1;
    double cruise_height_ = 1.0;
    double sector_range_ = M_PI / 3;
    double h_bottom_ = 0.0;
    double h_top_ = 2.0;

public:
    void setMapBridge(const MapBridge::Ptr& ptr);
    std::vector<double> solve(const Eigen::Vector3d & pos, const Eigen::Vector3d & vel, const Eigen::Vector3d & ctrl_p);
    Eigen::Vector4d guidedPseudoRaycast(const Eigen::Vector3d & pos, const Eigen::Vector3d & ctrl_p,
                                        float preset_radius = -1);

    Eigen::Vector2d singlePseudoRaycast(const Eigen::Vector2d & start, const Eigen::Vector2d & end, bool obs_free_mode);
    Eigen::Vector2d pseudoRaycast(int direction, const Eigen::Vector2d & start, double theta_init,
                                  double d_theta, double radius, const std::string & mode);

    [[nodiscard]] Corner getGridCorner(const Eigen::Vector2d & pos_2d) const;
    static Eigen::Vector2d getPeekCorner(const Eigen::Vector2d & start, const Eigen::Vector2d & end,
                                         const Corner & corners, int type);
};


#endif //BACKEND_OPTIMIZER_K_GPEP_H
