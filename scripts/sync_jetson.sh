#!/usr/bin/env bash
# Copy this project's verified source into its existing vehicle root only.
# No --delete, no environment/model reinstall, no sensor or motion startup.
set -euo pipefail
mapping_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mapping_target="${JETSON_SSH_TARGET:-tjark@192.168.10.102}"
mapping_remote_root=/home/tjark/yolo_rtabmap
mapping_preview=false
mapping_check=false
if [[ "${1:-}" == --help ]]; then
    echo '用法：bash scripts/sync_jetson.sh [--dry-run] [--check-software]'
    echo '默认只同步和核对语法；显式加 --check-software 才运行GPU软件检查。不启动传感器或车辆。'
    exit 0
fi
for mapping_arg in "$@"; do
    case "$mapping_arg" in
        --dry-run) mapping_preview=true ;;
        --check-software) mapping_check=true ;;
        *) exit 2 ;;
    esac
done
command -v rsync >/dev/null
mapping_list="$(mktemp /tmp/rtabmap-sync.XXXXXX)"
trap 'rm -f -- "$mapping_list"' EXIT
python3 - "$mapping_root" "$mapping_list" <<'PY'
from pathlib import Path
import ast,subprocess,sys
root=Path(sys.argv[1]);files=[]
runtime_scripts={'run_live.py','mapping_runtime.py','mapping_config.py',
 'ros_env.sh','ros_env_jetson.sh','sync_jetson.sh','run_camera.sh',
 'run_demo.py','inspect_bag.py','observe_ros.py',
 'read_semantic_output.py','inspect_live_interfaces.py'}
runtime_configs={'robot_mapping.yaml','live.yaml','camera_raw_only.yaml',
 'robot_mapping.rviz','jetson_constraints.txt','replay_cpu.yaml','semantic_indoor.rviz'}
for sub in ['semantic','launch','config','scripts']:
 for path in (root/sub).glob('*'):
  if not path.is_file() or path.suffix not in ('.py','.sh','.yaml','.rviz','.txt','.rules'):continue
  if sub=='scripts' and path.name not in runtime_scripts:continue
  if sub=='config' and path.name not in runtime_configs:continue
  if path.suffix=='.py':ast.parse(path.read_text(),filename=str(path))
  if path.suffix=='.sh':subprocess.run(['bash','-n',str(path)],check=True)
  files.append(str(path.relative_to(root)))
for name in ['AGENTS.md','README.md','LICENSE','requirements.txt','models/yolov8n-seg.pt','run.sh','stop.sh']:
 if (root/name).suffix=='.sh':subprocess.run(['bash','-n',str(root/name)],check=True)
 files.append(name)
Path(sys.argv[2]).write_text('\n'.join(files)+'\n')
PY
mapping_options=(-avc -s --files-from="$mapping_list")
if $mapping_preview; then mapping_options+=(--dry-run); fi
rsync "${mapping_options[@]}" "$mapping_root/" "$mapping_target:$mapping_remote_root/"
if ! $mapping_preview; then
    if $mapping_check; then
        ssh "$mapping_target" 'cd /home/tjark/yolo_rtabmap && bash run.sh --check'
        echo '源码同步及Jetson软件检查通过；尚未启动实车建图。'
    else
        echo '源码同步完成；未运行软件/GPU或实车测试。'
    fi
fi
