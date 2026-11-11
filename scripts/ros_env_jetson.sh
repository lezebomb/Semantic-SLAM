#!/usr/bin/env bash
# Independent Jetson entry. Never source unrelated robot workspaces or change dotfiles.
_deploy_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo 'ROS2 Humble is missing.' >&2
    return 1
fi
source /opt/ros/humble/setup.bash
if [ -f /etc/jetson-ros/ros2-env.sh ]; then source /etc/jetson-ros/ros2-env.sh; fi
export SEMANTIC_PYTHON="${SEMANTIC_PYTHON:-$_deploy_root/envs/yolo/bin/python}"
if [ ! -x "$SEMANTIC_PYTHON" ]; then
    echo "Deployment Python is missing: $SEMANTIC_PYTHON" >&2
    return 1
fi
export PATH="$(dirname -- "$SEMANTIC_PYTHON"):$PATH"
# Conda Python 3.10 loads system Python extension locations without global changes.
_semantic_site="$(dirname -- "$SEMANTIC_PYTHON")/../lib/python3.10/site-packages"
export PYTHONPATH="$_semantic_site:${PYTHONPATH:+$PYTHONPATH:}/usr/lib/python3/dist-packages:/usr/lib/python3.10/dist-packages"
export LD_LIBRARY_PATH="$_semantic_site/cusparselt/lib:$_semantic_site/nvidia/cusparselt/lib:${LD_LIBRARY_PATH:-}"
export RTABMAP_WS="$_deploy_root"
export YOLO_AUTOINSTALL=false PYTHONNOUSERSITE=1
export PIP_CONSTRAINT="$_deploy_root/config/jetson_constraints.txt"
export YOLO_CONFIG_DIR="$_deploy_root/logs/yolo_config"
export MPLCONFIGDIR="$_deploy_root/logs/matplotlib"
export XDG_CACHE_HOME="$_deploy_root/cache"
export CUDA_CACHE_PATH="$_deploy_root/cache/cuda" TORCH_HOME="$_deploy_root/cache/torch"
mkdir -p "$YOLO_CONFIG_DIR/Ultralytics" "$MPLCONFIGDIR" "$CUDA_CACHE_PATH" "$TORCH_HOME"
for _ros_pkg in cv_bridge message_filters sensor_msgs tf2_ros; do
    if ! /usr/bin/python3 -c "from ament_index_python.packages import get_package_prefix; get_package_prefix('$_ros_pkg')"; then
        echo "Missing ROS package: $_ros_pkg" >&2
        return 1
    fi
done
unset _deploy_root _ros_pkg _semantic_site
