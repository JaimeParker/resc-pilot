#!/usr/bin/env python

"""
This file is part of Resc-Pilot.

Copyright 2025 Zhaohong Liu, IRMV Lab, Shanghai Jiao Tong University, <https://www.sjtu.edu.cn/>
Developed by Zhaohong Liu <zhliu25 at outlook dot com>, <jaimefriedhelmzhao at gmail dot com>
for more information see <https://github.com/JaimeParker/resc-pilot>.
If you use this code, please cite the respective publications as
listed on the above website.

Resc-Pilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Resc-Pilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Resc-Pilot. If not, see <http://www.gnu.org/licenses/>.

Created by Zhaohong Liu on 24-10-21.
"""

import rospy
import pandas as pd
import rospkg
import os
import glob
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def extract_csv(csv_file, bias_z=None):
    # will add more extracted info later
    data = pd.read_csv(csv_file)

    pos_list = []

    if bias_z is not None:
        for index, row in data.iterrows():
            pos = [row['pos_x'], row['pos_y'], row['pos_z'] + bias_z]
            pos_list.append(pos)
    else:
        for index, row in data.iterrows():
            pos = [row['pos_x'], row['pos_y'], row['pos_z']]
            pos_list.append(pos)

    return pos_list


def init_node():
    rospy.init_node('global_trajectory_publisher', anonymous=True)


def create_marker_msg(pos_list, color=None):
    if color is None:
        color = [1.0, 0.0, 0.0]
    traj_marker = Marker()
    traj_marker.header.frame_id = "world"
    traj_marker.ns = "trajectory"
    traj_marker.id = 0
    traj_marker.type = Marker.LINE_STRIP
    traj_marker.action = Marker.ADD
    traj_marker.pose.orientation.w = 1.0
    traj_marker.scale.x = 0.1  # Line width
    traj_marker.color.a = 1.0  # Alpha
    traj_marker.color.r = color[0]  # Red
    traj_marker.color.g = color[1]  # Green
    traj_marker.color.b = color[2]  # Blue

    for pos in pos_list:
        point = Point()
        point.x = pos[0]
        point.y = pos[1]
        point.z = pos[2]
        traj_marker.points.append(point)

    return traj_marker


def find_latest_csv(path):
    csv_file = max(glob.glob(os.path.join(path, '*.csv')), key=os.path.getctime)
    return csv_file


if __name__ == '__main__':
    try:
        init_node()

        planner_marker_pub = rospy.Publisher('planner_trajectory', Marker, queue_size=10)
        proposed_marker_pub = rospy.Publisher('proposed_trajectory', Marker, queue_size=10)

        ros_pack = rospkg.RosPack()
        pkg_path = ros_pack.get_path('rviz_utils')

        pl_csv_path = os.path.join(pkg_path, 'recorder', 'sota-planner')
        pd_csv_path = os.path.join(pkg_path, 'recorder', 'proposed')

        pl_csv = find_latest_csv(pl_csv_path)
        pd_csv = find_latest_csv(pd_csv_path)

        planner_pos_list = extract_csv(pl_csv, bias_z=1.0)
        proposed_pos_list = extract_csv(pd_csv)

        planner_traj_marker = create_marker_msg(planner_pos_list, color=[1.0, 0.0, 0.0])
        proposed_traj_marker = create_marker_msg(proposed_pos_list, color=[0.0, 1.0, 0.0])

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            planner_marker_pub.publish(planner_traj_marker)
            proposed_marker_pub.publish(proposed_traj_marker)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


