#!/usr/bin/env bash
# Compatibility entry: all current physical tests use the complete RTAB-Map + YOLO pipeline.
set -e
mapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$mapping_root/run.sh" --data-only "$@"
