# This file is part of Resc-Pilot.
#
# Copyright 2025 Zhaohong Liu, IRMV Lab, Shanghai Jiao Tong University, <https://www.sjtu.edu.cn/>
# Developed by Zhaohong Liu <zhliu25 at outlook dot com>, <jaimefriedhelmzhao at gmail dot com>
# For more information see <https://github.com/JaimeParker/resc-pilot>.
# If you use this code, please cite the respective publications as
# listed on the above website.
#
# Resc-Pilot is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Resc-Pilot is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Resc-Pilot. If not, see <http://www.gnu.org/licenses/>.
#

# read rosbag file, plot the rate, velocity and other information to several videos

# color scheme: Barca color
# x: #A50044
# y: #004D98
# z: #EDBB00

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import cv2
from rospy import Time
import sys

if len(sys.argv) != 2:
    print("Usage: python3 rosbag_to_video_plot.py <bag_file>")
    sys.exit(1)
bag_file = sys.argv[1]

# video setting
frame_dpi = 200
fig_size_x = 8
fig_size_y = 2
frame_width = fig_size_x * frame_dpi
frame_height = fig_size_y * frame_dpi
frame_rate = 30

# color
x_color = '#A50044'
y_color = '#004D98'
z_color = '#EDBB00'

linewidth = 2.0
y_exceed_percent = 0.1
y_label_size = 12

bag = rosbag.Bag(bag_file)

velocity_topic = '/mavros/local_position/velocity_local'
rate_topic = '/mavros/imu/data'
att_cmd_topic = '/mavros/setpoint_raw/attitude'

vel_msg = bag.read_messages(topics=[velocity_topic])
rate_msg = bag.read_messages(topics=[rate_topic])
att_cmd_msg = bag.read_messages(topics=[att_cmd_topic])

vel_video_file = bag_file.replace('.bag', '') + '-vel.mp4'
rate_video_file = bag_file.replace('.bag', '') + '-rate.mp4'
cmd_video_file = bag_file.replace('.bag', '') + '-cmd.mp4'

times = []
vx = []
vy = []
vz = []

rate_times = []
wx = []
wy = []
wz = []

cmd_times = []
wx_req = []
wy_req = []
wz_req = []
thr_req = []

for topic, msg, t in vel_msg:
    timestamp = t.to_sec()
    times.append(timestamp)

    vx.append(msg.twist.linear.x)
    vy.append(msg.twist.linear.y)
    vz.append(msg.twist.linear.z)

for topic, msg, t in rate_msg:
    timestamp = t.to_sec()
    rate_times.append(timestamp)
    wx.append(msg.angular_velocity.x)
    wy.append(msg.angular_velocity.y)
    wz.append(msg.angular_velocity.z)

for topic, msg, t in att_cmd_msg:
    timestamp = t.to_sec()
    cmd_times.append(timestamp)
    wx_req.append(msg.body_rate.x)
    wy_req.append(msg.body_rate.y)
    wz_req.append(msg.body_rate.z)
    thr_req.append(msg.thrust)

bag.close()
print("bag read done")

start_time = min(times)
end_time = max(times)

times = [t - start_time for t in times]
rate_times = [t - start_time for t in rate_times]
cmd_times = [t - start_time for t in cmd_times]

# vel video creation
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(vel_video_file, fourcc, frame_rate, (frame_width, frame_height))
# rate video creation
rate_fourcc = cv2.VideoWriter_fourcc(*'mp4v')
rate_out = cv2.VideoWriter(rate_video_file, rate_fourcc, frame_rate, (frame_width, frame_height))
# cmd video creation
cmd_fourcc = cv2.VideoWriter_fourcc(*'mp4v')
cmd_out = cv2.VideoWriter(cmd_video_file, cmd_fourcc, frame_rate, (frame_width, frame_height))

# decide record time
preview_fig, preview_ax = plt.subplots(figsize=(12, 6))  # Larger figure for preview
preview_ax.plot(times, vx, label="vx", color=x_color, linewidth=linewidth)
preview_ax.plot(times, vy, label="vy", color=y_color, linewidth=linewidth)
preview_ax.plot(times, vz, label="vz", color=z_color, linewidth=linewidth)
preview_ax.set_xlabel('Time (s)')
preview_ax.set_ylabel('Velocity (m/s)')
preview_ax.grid(True, linestyle='--', alpha=0.5)
preview_ax.legend()
plt.show(block=False)

# get record duration time
start_record_time = float(input("Enter start time (in seconds): "))
end_record_time = float(input("Enter end time (in seconds): "))
plt.close(preview_fig)
duration = end_record_time - start_record_time
num_frames = int(duration * frame_rate)
start_record_time = max(0, min(start_record_time, max(times)))
end_record_time = max(start_record_time, min(end_record_time, max(times)))
frame_times = np.linspace(start_record_time, end_record_time, num_frames)

# create vel plot
fig, ax = plt.subplots(figsize=(fig_size_x, fig_size_y), dpi=frame_dpi)
fig.patch.set_facecolor('white')
ax.set_facecolor('white')
ax.set_xlim(start_record_time, end_record_time)
max_y = max(max(vx), max(vy), max(vz))
min_y = min(min(vx), min(vy), min(vz))
max_y = max_y + y_exceed_percent * (max_y - min_y)
min_y = min_y - y_exceed_percent * (max_y - min_y)
ax.set_ylim(min_y, max_y)
plt.rcParams['font.family'] = 'Helvetica'
# ax.set_xlabel('Time (s)')
# ax.set_xticks([])  # Remove x-axis ticks
ax.set_ylabel('Velocity (m/s)', fontsize=y_label_size)
ax.grid(True, linestyle='--', alpha=0.5)

line_vx, = ax.plot([], [], label="vx", color=x_color, linewidth=linewidth)
line_vy, = ax.plot([], [], label="vy", color=y_color, linewidth=linewidth)
line_vz, = ax.plot([], [], label="vz", color=z_color, linewidth=linewidth)

# create rate plot
rate_fig, rate_ax = plt.subplots(figsize=(fig_size_x, fig_size_y), dpi=frame_dpi)
rate_fig.patch.set_facecolor('white')
rate_ax.set_facecolor('white')
rate_ax.set_xlim(start_record_time, end_record_time)
rate_max_y = max(max(wx), max(wy), max(wz))
rate_min_y = min(min(wx), min(wy), min(wz))
rate_max_y = rate_max_y + y_exceed_percent * (rate_max_y - rate_min_y)
rate_min_y = rate_min_y - y_exceed_percent * (rate_max_y - rate_min_y)
rate_ax.set_ylim(rate_min_y, rate_max_y)
# rate_ax.set_xticks([])
rate_ax.set_ylabel('Rate (rad/s)', fontsize=y_label_size)
rate_ax.grid(True, linestyle='--', alpha=0.5)

line_wx, = rate_ax.plot([], [], color=x_color, linewidth=linewidth)
line_wy, = rate_ax.plot([], [], color=y_color, linewidth=linewidth)
line_wz, = rate_ax.plot([], [], color=z_color, linewidth=linewidth)

# create cmd plot
cmd_fig, cmd_ax = plt.subplots(figsize=(fig_size_x, fig_size_y), dpi=frame_dpi)
cmd_ax2 = cmd_ax.twinx()
cmd_fig.patch.set_facecolor('white')
cmd_ax.set_facecolor('white')
cmd_ax2.set_facecolor('white')
cmd_ax.set_xlim(start_record_time, end_record_time)
cmd_max_y = max(max(wx_req), max(wy_req), max(wz_req))
cmd_min_y = min(min(wx_req), min(wy_req), min(wz_req))
cmd_max_y = cmd_max_y + y_exceed_percent * (cmd_max_y - cmd_min_y)
cmd_min_y = cmd_min_y - y_exceed_percent * (cmd_max_y - cmd_min_y)
cmd_ax.set_ylim(cmd_min_y, cmd_max_y)
# cmd_ax.set_xticks([])
cmd_ax.set_xlabel('Time (s)', fontsize=y_label_size)
cmd_ax.set_ylabel('Rate Req (rad/s)', fontsize=y_label_size)
cmd_ax.grid(True, linestyle='--', alpha=0.5)

thr_max_y = max(thr_req)
thr_min_y = min(thr_req)
thr_max_y = thr_max_y + y_exceed_percent * (thr_max_y - thr_min_y)
thr_min_y = thr_min_y - y_exceed_percent * (thr_max_y - thr_min_y)
cmd_ax2.set_ylim(thr_min_y, thr_max_y)
cmd_ax2.set_ylabel('Throttle', fontsize=y_label_size)

line_wx_req, = cmd_ax.plot([], [], color=x_color, linewidth=linewidth)
line_wy_req, = cmd_ax.plot([], [], color=y_color, linewidth=linewidth)
line_wz_req, = cmd_ax.plot([], [], color=z_color, linewidth=linewidth)
line_thr_req, = cmd_ax2.plot([], [], color='gray', linewidth=linewidth, linestyle='--')

print("processing vel video...")
for i in range(num_frames):
    current_time = frame_times[i]
    closest_idx = np.argmin(np.abs(np.array(times) - current_time))
    
    line_vx.set_data(times[:closest_idx+1], vx[:closest_idx+1])
    line_vy.set_data(times[:closest_idx+1], vy[:closest_idx+1])
    line_vz.set_data(times[:closest_idx+1], vz[:closest_idx+1])

    plt.draw()
    plt.pause(0.01)

    fig.canvas.draw()
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    image = image.reshape((frame_height, frame_width, 3))
    out.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

out.release()
plt.close(fig)
print("vel video saved!")

print("processing rate video...")
for i in range(num_frames):
    current_time = frame_times[i]
    closest_idx = np.argmin(np.abs(np.array(rate_times) - current_time))
    
    line_wx.set_data(rate_times[:closest_idx+1], wx[:closest_idx+1])
    line_wy.set_data(rate_times[:closest_idx+1], wy[:closest_idx+1])
    line_wz.set_data(rate_times[:closest_idx+1], wz[:closest_idx+1])

    plt.draw()
    plt.pause(0.01)

    rate_fig.canvas.draw()
    rate_image = np.frombuffer(rate_fig.canvas.tostring_rgb(), dtype=np.uint8)
    rate_image = rate_image.reshape((frame_height, frame_width, 3))
    rate_out.write(cv2.cvtColor(rate_image, cv2.COLOR_RGB2BGR))

print("processing cmd video...")
for i in range(num_frames):
    current_time = frame_times[i]
    closest_idx = np.argmin(np.abs(np.array(cmd_times) - current_time))
    
    line_wx_req.set_data(cmd_times[:closest_idx+1], wx_req[:closest_idx+1])
    line_wy_req.set_data(cmd_times[:closest_idx+1], wy_req[:closest_idx+1])
    line_wz_req.set_data(cmd_times[:closest_idx+1], wz_req[:closest_idx+1])
    line_thr_req.set_data(cmd_times[:closest_idx+1], thr_req[:closest_idx+1])

    plt.draw()
    plt.pause(0.01)

    cmd_fig.canvas.draw()
    cmd_image = np.frombuffer(cmd_fig.canvas.tostring_rgb(), dtype=np.uint8)
    cmd_image = cmd_image.reshape((frame_height, frame_width, 3))
    cmd_out.write(cv2.cvtColor(cmd_image, cv2.COLOR_RGB2BGR))

rate_out.release()
plt.close(rate_fig)
print("rate video saved!")

cv2.destroyAllWindows()
