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

#ifndef RVIZ_UTILS_RVIZPARAMS_H
#define RVIZ_UTILS_RVIZPARAMS_H

#include <string>

class RvizParams {
public:
    static std::string pose_marker_pub_topic_;

    static std::string mesh_resource_path_;

    static std::string world_frame_id_;

    static constexpr int sub_queue_size_ = 5;
    static constexpr int pub_queue_size_ = 1;

    static constexpr double robot_scale_x_ = 1.0;
    static constexpr double robot_scale_y_ = 1.0;
    static constexpr double robot_scale_z_ = 1.0;

    static constexpr double rviz_frequency_ = 10.0;
};



#endif //RVIZ_UTILS_RVIZPARAMS_H
