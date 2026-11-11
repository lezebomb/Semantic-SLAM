#!/usr/bin/env bash
# Compatibility alias; stop.sh owns PID checks and graceful shutdown.
set -e
mapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$mapping_root/stop.sh" "$@"
