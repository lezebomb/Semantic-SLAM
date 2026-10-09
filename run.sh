#!/usr/bin/env bash
set -e
SEMANTIC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SEMANTIC_ROOT/scripts/ros_env.sh"
if [ ! -x "$SEMANTIC_ROOT/.venv/bin/python" ]; then
    echo "Missing .venv; run ./setup.sh first." >&2
    exit 1
fi
exec "$SEMANTIC_ROOT/.venv/bin/python" "$SEMANTIC_ROOT/scripts/run_demo.py" "$@"
