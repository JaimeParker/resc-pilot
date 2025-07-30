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
 * Created by Zhaohong Liu on 24-10-19.
*/

#ifndef RVIZ_UTILS_PATHVISUALIZATION_H
#define RVIZ_UTILS_PATHVISUALIZATION_H

#include <geometry_msgs/Point.h>

#include "MarkerHandler.h"
#include "VisualizationColor.h"

using VC = VisualizationColor;

class PathVisualization : public MarkerHandler {
public:
    PathVisualization(const ros::NodeHandle& nh, const std::string& frame_id)
        : MarkerHandler(frame_id, nh) {}
    void updateMarker() override;
    void init() override;
    void publish() override;
    void setPath(const std::vector<Eigen::Vector3d>& path);
    void clearPath();
    void path2MarkerPoints();
private:
    std::vector<Eigen::Vector3d> path_;
    ros::Publisher path_rviz_pub_;
    std::string path_rviz_topic_ = "path_rviz";

    double cruise_height_ = 1.0;
};


#endif //RVIZ_UTILS_PATHVISUALIZATION_H
