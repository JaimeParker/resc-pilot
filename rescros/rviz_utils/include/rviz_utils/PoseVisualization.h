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
 * Created by Zhaohong Liu on 24-9-18.
*/

#ifndef RVIZ_UTILS_POSEVISUALIZATION_H
#define RVIZ_UTILS_POSEVISUALIZATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>

#include "MarkerHandler.h"
#include "RvizParams.h"
#include "VisualizationColor.h"

using RP = RvizParams;
using VC = VisualizationColor;

class PoseMarkerHandler : public MarkerHandler {
public:
    PoseMarkerHandler(const ros::NodeHandle& nh, const std::string& frame_id)
        : MarkerHandler(frame_id, nh) {
        // color scheme from: http://lcpmgh.com/colors/
        // hex: #EC817E
        x_axis_color_.a = 1.0;
        x_axis_color_.r = 0.9254901960784314;
        x_axis_color_.g = 0.5058823529411764;
        x_axis_color_.b = 0.49411764705882355;
        // hex: #71BFB2
        y_axis_color_.a = 1.0;
        y_axis_color_.r = 0.44313725490196076;
        y_axis_color_.g = 0.7490196078431373;
        y_axis_color_.b = 0.6980392156862745;
        // hex: #237B9F
        z_axis_color_.a = 1.0;
        z_axis_color_.r = 0.13725490196078433;
        z_axis_color_.g = 0.4823529411764706;
        z_axis_color_.b = 0.6235294117647059;
        // hex: #074C9B
        drone_color_.a = 1.0;
        drone_color_.r = 0.027450980392156862;
        drone_color_.g = 0.2980392156862745;
        drone_color_.b = 0.6078431372549019;
    }

    void updateMarker() override;
    void init() override;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publish() override;

private:
    /* ros utils */
    ros::Subscriber pose_sub_;
    ros::Publisher pose_marker_pub_;
    ros::Publisher heading_marker_pub_;
    ros::Publisher axis_marker_pub_;

    geometry_msgs::PoseStamped pose_msg_;
    visualization_msgs::Marker heading_marker_;
    visualization_msgs::Marker x_axis_marker_;
    visualization_msgs::Marker y_axis_marker_;
    visualization_msgs::Marker z_axis_marker_;

    std::string pose_sub_topic_ = "/mavros/local_position/pose";
    std::string heading_marker_pub_topic_ = "/heading_marker";
    std::string drone_att_in_axis_pub_topic_ = "/drone_att_marker";

    /* params */
    std_msgs::ColorRGBA x_axis_color_;
    std_msgs::ColorRGBA y_axis_color_;
    std_msgs::ColorRGBA z_axis_color_;
    std_msgs::ColorRGBA drone_color_;
};


#endif //RVIZ_UTILS_POSEVISUALIZATION_H
