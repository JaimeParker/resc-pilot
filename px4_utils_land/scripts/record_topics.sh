#!/bin/bash

# Get the current date and time to use as the rosbag file name
current_time=$(date +%Y-%m-%d_%H-%M-%S)

# Define the topics to be recorded
topics=(
  "/mavros/local_position/pose"
  "/mavros/local_position/velocity"
  "/mavros/local_position/velocity_local"
  "/mavros/local_position/velocity_body"
  "/mavros/global_position/pose"
  "/scan"
  "/mavros/setpoint_raw/attitude"
  "/mavros/imu/body_rate"
  "/mavros/imu/data"
  "/search/wpt0"
  "/search/wpt1"
)

# Record the topics and save the rosbag file with the current time as the name
rosbag record -O "$current_time.bag" "${topics[@]}"

