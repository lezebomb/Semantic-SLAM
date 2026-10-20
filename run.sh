#!/usr/bin/env bash
set -e
SEMANTIC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GSETTINGS_SCHEMA_DIR="${GSETTINGS_SCHEMA_DIR:-/usr/share/glib-2.0/schemas}"
export YOLO_AUTOINSTALL=false
source "$SEMANTIC_ROOT/scripts/ros_env.sh"
if [ ! -x "$SEMANTIC_ROOT/.venv/bin/python" ]; then
    echo "Missing .venv; run ./setup.sh first." >&2
    exit 1
fi
exec "$SEMANTIC_ROOT/.venv/bin/python" "$SEMANTIC_ROOT/scripts/run_demo.py" "$@"
