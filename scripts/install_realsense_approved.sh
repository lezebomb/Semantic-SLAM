#!/usr/bin/env bash
# Execute only after user approval for both the seven new packages and this D435 rule.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
packages=(ros-humble-realsense2-camera ros-humble-realsense2-description)
apt-get -s --no-install-recommends install "${packages[@]}" > "$root/logs/realsense_apt_plan.txt"
python3 - "$root/logs/realsense_apt_plan.txt" <<'PY'
import pathlib,re,sys
s=pathlib.Path(sys.argv[1]).read_text()
m=re.search(r'(\d+) upgraded, (\d+) newly installed, (\d+) to remove',s)
if not m or int(m[1]) or int(m[3]) or int(m[2])>7:
    sys.exit('STOP: camera installation plan differs; review required.')
print(m[0])
PY
rule=/etc/udev/rules.d/99-robot-deploy-d435.rules
if [ -e "$rule" ] && ! cmp -s "$root/config/99-robot-deploy-d435.rules" "$rule"; then
    echo 'STOP: existing D435 deployment rule differs; not overwriting.' >&2
    exit 1
fi
sudo apt-get --no-upgrade --no-remove --no-install-recommends -y install "${packages[@]}" 2>&1 | tee "$root/logs/realsense_apt_install.log"
sudo install -m 0644 "$root/config/99-robot-deploy-d435.rules" "$rule"
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b07
date -Is > "$root/logs/realsense_install.completed"
