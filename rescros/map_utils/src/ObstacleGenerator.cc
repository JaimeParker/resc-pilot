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
 * Created by Zhaohong Liu on 24-9-19.
*/

#include "map_utils/ObstacleGenerator.h"

void ObstacleGenerator::setRectPcl(double x, double y, double length, double width, double height,
                                   pcl::PointCloud<pcl::PointXYZ> & cloud, bool rd_h) const {
    // ref: https://github.com/HKUST-Aerial-Robotics/Fast-Planner/blob/master/uav_simulator/map_generator/
    pcl::PointXYZ pt;

    // Calculate the boundaries of the rectangle
    double x_start = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
    double x_end = floor((x + length) / resolution_) * resolution_ + resolution_ / 2.0;
    double y_start = floor(y / resolution_) * resolution_ + resolution_ / 2.0;
    double y_end = floor((y + width) / resolution_) * resolution_ + resolution_ / 2.0;

    int x_widNum = ceil((x_end - x_start) / resolution_);
    int y_widNum = ceil((y_end - y_start) / resolution_);

    // create a random number from 0 to 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    for (int r = 0; r < x_widNum; r++)
        for (int s = 0; s < y_widNum; s++) {
            double rd_height = height;
            if (rd_h) {
                rd_height = dis(gen) + height;
            }
            int heiNum = ceil(rd_height / resolution_);
            for (int t = ground_height_; t < heiNum; t++) {
                pt.x = static_cast<float>(x_start + (r + 0.5) * resolution_ + 1e-2);
                pt.y = static_cast<float>(y_start + (s + 0.5) * resolution_ + 1e-2);
                pt.z = static_cast<float>((t + 0.5) * resolution_ + 1e-2);
                cloud.points.emplace_back(pt);
            }
        }
}
