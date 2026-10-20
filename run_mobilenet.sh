#!/usr/bin/env bash
set -e
DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$DEPLOY_ROOT/run.sh" --model "$DEPLOY_ROOT/checkpoints/mobilenetv3-small-yolov8n-seg.pt" --allow-empty "$@"
