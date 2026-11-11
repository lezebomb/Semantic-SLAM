#!/usr/bin/env bash
# D435 only. Does not start chassis, arm, odometry, SLAM, navigation or static mount TF.
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f /etc/jetson-ros/ros2-env.sh ]]; then source /etc/jetson-ros/ros2-env.sh; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_HOME="$root/logs/d435_ros_home" ROS_LOG_DIR="$root/logs/d435_ros"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
exec ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true \
    align_depth.enable:=true enable_sync:=true rgb_camera.color_profile:=640,480,15 \
    depth_module.depth_profile:=640,480,15 initial_reset:=false wait_for_device_timeout:=10.0 config_file:="$root/config/camera_raw_only.yaml" "$@"
