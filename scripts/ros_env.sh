#!/usr/bin/env bash
# Project runtime: x86 local environment or the existing ARM64 Jetson environment.
if [[ "$(uname -m)" == "aarch64" ]]; then
    _semantic_env_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    source "$_semantic_env_dir/ros_env_jetson.sh"
    return $?
fi
# Source this file from run.sh/setup.sh; the semantic nodes need Humble Python 3.10.
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    return 1
fi
source "$ROS_SETUP"
RTABMAP_WS="${RTABMAP_WS:-$HOME/ros2}"
if [ -f "$RTABMAP_WS/install/setup.bash" ]; then
    source "$RTABMAP_WS/install/setup.bash"
fi
export RTABMAP_WS
for pkg in cv_bridge message_filters image_transport compressed_image_transport compressed_depth_image_transport rtabmap_demos rtabmap_slam rtabmap_sync rtabmap_util rtabmap_msgs rtabmap_rviz_plugins tf2_ros rviz2 rosbag2_transport; do
    if ! ros2 pkg prefix "$pkg" >/dev/null; then
        echo "Missing ROS package: $pkg. Follow README.md or set RTABMAP_WS." >&2
        return 1
    fi
done
RTABMAP_SHARE="$(ros2 pkg prefix --share rtabmap_demos)"
if [ ! -f "$RTABMAP_SHARE/launch/robot_mapping_demo.launch.py" ]; then
    echo "Missing robot_mapping_demo.launch.py; use the RTAB-Map revision in README.md." >&2
    return 1
fi

_semantic_local_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export SEMANTIC_PYTHON="${SEMANTIC_PYTHON:-$_semantic_local_root/.venv/bin/python}"
unset _semantic_local_root
