#!/usr/bin/env bash
# Known Indoor rosbag replay on Jetson: no real camera, LiDAR or chassis required.
set -e
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$ROOT/run.sh" --official --no-rviz --duration 120 "$@"
