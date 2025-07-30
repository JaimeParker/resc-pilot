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
 * Created by Zhaohong Liu on 24-10-21.
*/

#ifndef RVIZ_UTILS_CTRLPOINTVISUALIZATION_H
#define RVIZ_UTILS_CTRLPOINTVISUALIZATION_H

#include <geometry_msgs/Point.h>

#include "MarkerHandler.h"
#include "VisualizationColor.h"

using VC = VisualizationColor;

class CtrlPtMarkerHandler : public MarkerHandler {
public:
    CtrlPtMarkerHandler(const ros::NodeHandle& nh, const std::string& frame_id)
            : MarkerHandler(frame_id, nh) {}

    void updateMarker() override;
    void init() override;
    void publish() override;
    void ctrlPt1Callback(const geometry_msgs::Point::ConstPtr& msg);
    void ctrlPt2Callback(const geometry_msgs::Point::ConstPtr& msg);
private:
    bool is_initialized_ = false;
    visualization_msgs::Marker ctrl_pt_second_marker_;
    ros::Publisher ctrl_pt_rviz_pub_;
    std::string ctrl_pt_rviz_topic_ = "ctrl_pt_rviz";
    ros::Subscriber ctrl_pt_1st_sub_;
    ros::Subscriber ctrl_pt_2nd_sub_;
    std::string ctrl_pt_1st_topic_ = "/search/wpt0";
    std::string ctrl_pt_2nd_topic_ = "/search/wpt1";
};


#endif //RVIZ_UTILS_CTRLPOINTVISUALIZATION_H
