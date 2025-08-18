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

import cv2
import rosbag
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import os


def create_sigmoid_lut(k=0.1, c=127):
    """
    Create a sigmoid LUT to map input values [0, 255] to output values [0, 255].
    Args:
        k: Steepness of the sigmoid curve (higher values make the transition sharper).
        c: Center of the sigmoid curve (input value around which the sigmoid changes most rapidly).
    Returns:
        lut: A numpy array of shape (256,) containing the LUT.
    """
    x = np.arange(256, dtype=np.float32)
    lut = 255 / (1 + np.exp(-k * (x - c)))  # Sigmoid function
    return lut.astype(np.uint8)


bag_file = '2024-12-27-video.bag'

depth_min = 0
depth_max = 255
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = 30

# Update topics for compressed images
rgb_topic = '/camera/color/image_raw/compressed'
depth_topic = '/camera/depth/image_rect_raw/compressed'

rosbag_name = os.path.splitext(os.path.basename(bag_file))[0]
rgb_video = f'{rosbag_name}-rgb.mp4'
depth_video = f'{rosbag_name}-depth.mp4'

lut_k = 0.02  # Steepness of the sigmoid (adjust for more/less contrast)
lut_c = 200   # Center point of the sigmoid
lut = create_sigmoid_lut(k=lut_k, c=lut_c)

bridge = CvBridge()
bag = rosbag.Bag(bag_file, 'r')

# Get dimensions from first frames
for topic, msg, t in bag.read_messages(topics=[rgb_topic]):
    rgb_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")  # Changed method
    height, width, _ = rgb_image.shape
    break

for topic, msg, t in bag.read_messages(topics=[depth_topic]):
    depth_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")  # Changed method
    depth_height, depth_width = depth_image.shape
    break

rgb_writer = cv2.VideoWriter(rgb_video, fourcc, fps, (width, height))
depth_writer = cv2.VideoWriter(depth_video, fourcc, fps, (depth_width, depth_height))

print(f"Converting {rgb_topic} to {rgb_video} and {depth_topic} to {depth_video}...")

global_min = float('inf')
global_max = float('-inf')

for topic, msg, t in bag.read_messages(topics=[rgb_topic, depth_topic]):
    if topic == rgb_topic:
        rgb_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")  # Changed method
        rgb_writer.write(rgb_image)
    elif topic == depth_topic:
        depth_image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        frame_min = np.min(depth_image)
        frame_max = np.max(depth_image)
        global_min = min(global_min, frame_min)
        global_max = max(global_max, frame_max)

        depth_image_clipped = np.clip(depth_image, depth_min, depth_max)
        depth_image_mapped = cv2.LUT(depth_image_clipped.astype(np.uint8), lut)
        depth_image_normalized = ((depth_image_mapped - depth_min) / (depth_max - depth_min) * 255).astype('uint8')
        depth_image_3ch = cv2.cvtColor(depth_image_normalized, cv2.COLOR_GRAY2BGR)
        depth_writer.write(depth_image_3ch)

print(f"Global depth range across all frames: min={global_min}, max={global_max}")

print("Convert finished!")

rgb_writer.release()
depth_writer.release()
bag.close()

print(f"Videos saved: {rgb_video}, {depth_video}")