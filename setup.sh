#!/usr/bin/env bash
set -e
if [ "$(uname -m)" != x86_64 ]; then
    echo 'This installer is for x86_64 CPU wheels. For Jetson see docs/JETSON.md; keep its JetPack-matched torch/torchvision.' >&2
    exit 1
fi
SEMANTIC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SEMANTIC_ROOT/scripts/ros_env.sh"
if [ ! -x "$SEMANTIC_ROOT/.venv/bin/python" ]; then
    /usr/bin/python3 -m venv --system-site-packages "$SEMANTIC_ROOT/.venv"
fi
export YOLO_CONFIG_DIR="$SEMANTIC_ROOT/artifacts/setup_yolo_config"
export MPLCONFIGDIR="$SEMANTIC_ROOT/artifacts/setup_matplotlib"
mkdir -p "$SEMANTIC_ROOT/models" "$SEMANTIC_ROOT/artifacts"
if ! "$SEMANTIC_ROOT/.venv/bin/python" -c 'import torch, torchvision; assert torch.__version__ == "2.5.1+cpu"; assert torchvision.__version__ == "0.20.1+cpu"' >/dev/null 2>&1; then
    "$SEMANTIC_ROOT/.venv/bin/python" -m pip install --disable-pip-version-check --no-cache-dir \
        'torch==2.5.1+cpu' 'torchvision==0.20.1+cpu' --index-url https://download.pytorch.org/whl/cpu
fi
"$SEMANTIC_ROOT/.venv/bin/python" -m pip install --disable-pip-version-check --no-cache-dir -r "$SEMANTIC_ROOT/requirements.txt"
MODEL_SHA256=a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c
if [ ! -s "$SEMANTIC_ROOT/models/yolov8n-seg.pt" ]; then
    curl -fL --retry 2 --connect-timeout 20 --max-time 300 \
        https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-seg.pt \
        -o "$SEMANTIC_ROOT/models/yolov8n-seg.pt.part"
    printf '%s  %s\n' "$MODEL_SHA256" "$SEMANTIC_ROOT/models/yolov8n-seg.pt.part" | sha256sum -c -
    mv "$SEMANTIC_ROOT/models/yolov8n-seg.pt.part" "$SEMANTIC_ROOT/models/yolov8n-seg.pt"
fi
printf '%s  %s\n' "$MODEL_SHA256" "$SEMANTIC_ROOT/models/yolov8n-seg.pt" | sha256sum -c -
"$SEMANTIC_ROOT/.venv/bin/python" - <<'PY'
import cv2, cv_bridge, message_filters, numpy, rclpy, torch, torchvision, ultralytics
assert numpy.__version__ == '1.26.4'
assert cv2.__version__ == '4.10.0'
assert ultralytics.__version__ == '8.3.221'
print('Ready:', 'torch', torch.__version__, 'torchvision', torchvision.__version__,
      'numpy', numpy.__version__, 'opencv', cv2.__version__, 'ultralytics', ultralytics.__version__)
PY
