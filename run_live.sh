#!/usr/bin/env bash
set -e
DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$DEPLOY_ROOT/scripts/ros_env.sh"
export SEMANTIC_PYTHON="${SEMANTIC_PYTHON:-$DEPLOY_ROOT/.venv/bin/python}"
if [ ! -x "$SEMANTIC_PYTHON" ]; then
    echo 'Set SEMANTIC_PYTHON to a ROS-aware Python environment; see docs/JETSON.md.' >&2
    exit 1
fi
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" YOLO_AUTOINSTALL=false
export GSETTINGS_SCHEMA_DIR="${GSETTINGS_SCHEMA_DIR:-/usr/share/glib-2.0/schemas}"
export PYTHONUNBUFFERED=1
exec ros2 launch "$DEPLOY_ROOT/launch/live_semantic.launch.py" "$@"
