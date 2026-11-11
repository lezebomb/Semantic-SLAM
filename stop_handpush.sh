#!/usr/bin/env bash
set -e
mapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$mapping_root/stop.sh" "$@"
