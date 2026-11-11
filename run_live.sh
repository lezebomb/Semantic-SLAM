#!/usr/bin/env bash
# Compatibility alias; run.sh owns all modes and process management.
set -e
mapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$mapping_root/run.sh" "$@"
