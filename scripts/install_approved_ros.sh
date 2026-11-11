#!/usr/bin/env bash
# Approved by the user on 2026-10-23: these six packages and their new dependencies.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
packages=(ros-humble-rtabmap-slam ros-humble-rtabmap-sync ros-humble-rtabmap-util
          ros-humble-rtabmap-msgs ros-humble-compressed-image-transport
          ros-humble-compressed-depth-image-transport)
mkdir -p "$root/logs"
apt-get -s --no-install-recommends install "${packages[@]}" > "$root/logs/approved_apt_plan.txt"
python3 - "$root/logs/approved_apt_plan.txt" <<'PY'
import pathlib,re,sys
s=pathlib.Path(sys.argv[1]).read_text()
m=re.search(r'(\d+) upgraded, (\d+) newly installed, (\d+) to remove',s)
if not m or int(m[1]) or int(m[3]) or int(m[2])>41:
    sys.exit('STOP: installation plan changed; review required. See approved_apt_plan.txt')
print(m[0])
PY
sudo apt-get --no-upgrade --no-remove --no-install-recommends -y install "${packages[@]}" 2>&1 | tee "$root/logs/approved_apt_install.log"
date -Is > "$root/logs/approved_apt_install.completed"
