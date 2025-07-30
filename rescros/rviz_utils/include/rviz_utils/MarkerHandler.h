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

#ifndef RVIZ_UTILS_MARKERHANDLER_H
#define RVIZ_UTILS_MARKERHANDLER_H

#include <utility>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>

class MarkerHandler {
public:
    virtual ~MarkerHandler() = default;

    [[nodiscard]] virtual visualization_msgs::Marker getMarker() const {
        return marker_;
    }
    virtual void updateMarker() = 0;
    virtual void init() = 0;
    virtual void publish() = 0;

protected:
    /**
     * @brief Protected constrictor so that only derived classes can instantiate with handler and frame id
     * @param frame_id std::string, frame id for the marker, no '/' for ubuntu 20.04
     * @param nh ros::NodeHandler, to subscribe/publish if needed
     */
    MarkerHandler(std::string  frame_id, const ros::NodeHandle& nh)
            : nh_(nh), frame_id_(std::move(frame_id)) {}

    /**
     * @brief Set common properties of a marker, such as frame id and timestamp.
     * Virtual so that derived classes can override if needed.
     * @param marker visualization_msgs::Marker, the marker to set properties
     */
    virtual void setCommonMarkerProperties(visualization_msgs::Marker& marker) {
        marker.header.frame_id = frame_id_;
        marker.color.a = 1.0;  // Default alpha value
    }

    bool initialized_ = false;
    ros::NodeHandle nh_;
    std::string frame_id_;
    visualization_msgs::Marker marker_;
};

#endif //RVIZ_UTILS_MARKERHANDLER_H
