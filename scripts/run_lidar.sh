#!/usr/bin/env bash
# Receive existing LakiBeam UDP data only. Does not launch chassis/TF/navigation.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
if [[ -f /etc/jetson-ros/ros2-env.sh ]]; then source /etc/jetson-ros/ros2-env.sh; fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
export ROS_HOME="$ROOT/logs/lidar_ros_home"
export ROS_LOG_DIR="$ROOT/logs/lidar_ros"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
if [[ -n $(ss -H -lun 'sport = :2368') ]]; then
    echo 'UDP 2368 is already in use; leave the existing receiver untouched.' >&2
    exit 1
fi
exec "$ROOT/sensor_ws/install/lakibeam1/lib/lakibeam1/lakibeam1_scan_node" \
    --ros-args --params-file "$ROOT/config/lakibeam_passive.yaml"
