#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/ros2_ws/install/setup.bash

# Check if rosbag directory has files
if [ -d /root/rosbag ] && [ -n "$(ls -A /root/rosbag 2>/dev/null)" ]; then
    echo "Found rosbag files, starting playback in loop mode..."
    ros2 bag play /root/rosbag --loop &
else
    echo "No rosbag files found in /root/rosbag, skipping playback..."
fi

# Prepare mosaic config argument
MOSAIC_CONFIG_ARG=""
if [ -n "${MOSAIC_CONFIG_PATH}" ]; then
    echo "Using config file from environment variable: ${MOSAIC_CONFIG_PATH}"
    MOSAIC_CONFIG_ARG="mosaic_config:=${MOSAIC_CONFIG_PATH}"
elif [ -f /root/mosaic_config/config.yaml ]; then
    echo "Using default config file: /root/mosaic_config/config.yaml"
    MOSAIC_CONFIG_ARG="mosaic_config:=/root/mosaic_config/config.yaml"
else
    echo "No config file provided, using launch file default"
fi

# Start mosaic launch file in foreground
if [ -n "${MOSAIC_CONFIG_ARG}" ]; then
    ros2 launch mosaic-ros2-bringup mosaic_bringup_launch.py ${MOSAIC_CONFIG_ARG}
else
    ros2 launch mosaic-ros2-bringup mosaic_bringup_launch.py
fi