#!/usr/bin/env bash
# One public entry: vehicle session, sensor replay, or explicit official demo.
set -e
mapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    exec /usr/bin/python3 "$mapping_root/scripts/run_live.py" --help
fi
mapping_official=false
if [[ "${1:-}" == --official ]]; then mapping_official=true; shift; fi
mapping_domain="${ROS_DOMAIN_ID:-}"
mapping_replay=false
for mapping_arg in "$@"; do
    if [[ "$mapping_arg" == --replay || "$mapping_arg" == --replay=* ]]; then mapping_replay=true; fi
done
source "$mapping_root/scripts/ros_env.sh"
unset QT_QPA_PLATFORM_PLUGIN_PATH QT_QPA_FONTDIR
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" YOLO_AUTOINSTALL=false PYTHONUNBUFFERED=1
export GSETTINGS_SCHEMA_DIR="${GSETTINGS_SCHEMA_DIR:-/usr/share/glib-2.0/schemas}"
if $mapping_official; then
    exec "$SEMANTIC_PYTHON" "$mapping_root/scripts/run_demo.py" "$@"
fi
if $mapping_replay; then
    if [[ -z "$mapping_domain" || "$mapping_domain" == 0 ]]; then mapping_domain=77; fi
    export ROS_DOMAIN_ID="$mapping_domain" ROS_LOCALHOST_ONLY=1
else
    export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
fi
exec /usr/bin/python3 "$mapping_root/scripts/run_live.py" "$@"
