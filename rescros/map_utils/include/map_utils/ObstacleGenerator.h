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

#ifndef MAP_UTILS_OBSTACLEGENERATOR_H
#define MAP_UTILS_OBSTACLEGENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

class ObstacleGenerator {
public:
    void setResolution(double res) { resolution_ = res; }
    void setRectPcl(double x, double y, double length, double width, double height,
                    pcl::PointCloud<pcl::PointXYZ> & cloud, bool rd_h = true) const;
private:
    double resolution_ = 0.1;
    int ground_height_ = -1;  // ground_height_ * resolution_
};


#endif //MAP_UTILS_OBSTACLEGENERATOR_H
