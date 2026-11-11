#!/usr/bin/env python3
"""Read-only dependency/hash/forward check; the zero input is not a recognition test."""
import argparse
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='mobilenet', help='baseline, mobilenet, or checkpoint path')
    parser.add_argument('--device', default='cpu', choices=['cpu', '0'])
    parser.add_argument('--threads', type=int, default=2)
    args = parser.parse_args()
    if args.threads < 1:
        parser.error('--threads must be positive')
    os.environ.setdefault('YOLO_CONFIG_DIR', str(ROOT / 'artifacts/yolo_config'))
    Path(os.environ['YOLO_CONFIG_DIR']).mkdir(parents=True, exist_ok=True)
    os.environ['YOLO_AUTOINSTALL'] = 'false'
    import numpy as np
    import torch
    import torchvision
    from ultralytics import YOLO
    manifest = json.loads((ROOT / 'checkpoints/manifest.json').read_text())
    record = manifest.get(args.model)
    path = ROOT / record['path'] if record else Path(args.model).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f'Missing checkpoint: {path}; see checkpoints/README.md')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if record and digest != record['sha256']:
        raise SystemExit('Checkpoint hash mismatch; refusing to load')
    if importlib.metadata.version('ultralytics') != '8.3.221' or int(np.__version__.split('.')[0]) != 1:
        raise SystemExit('Expected ultralytics8.3.221 and NumPy1.x for the ROS Humble path')
    if args.device == '0' and not torch.cuda.is_available():
        raise SystemExit('CUDA unavailable; retain/install the JetPack-matched NVIDIA torch/torchvision pair')
    device = torch.device('cuda:0' if args.device == '0' else 'cpu')
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    # Exercise the actual NMS extension on the requested device.
    boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 9., 9.]], device=device)
    assert len(torchvision.ops.nms(boxes, torch.tensor([.9, .8], device=device), .5)) == 1
    model = YOLO(str(path), task='segment').model.to(device).float().eval()
    if len(model.names) != 80:
        raise SystemExit('Expected COCO80 segmentation output')
    with torch.inference_mode():
        prediction, extra = model(torch.zeros(1, 3, 640, 640, device=device))
    assert tuple(prediction.shape) == (1, 116, 8400) and tuple(extra[2].shape) == (1, 32, 160, 160)
    assert torch.isfinite(prediction).all() and torch.isfinite(extra[2]).all()
    print(json.dumps(dict(check='passed', checkpoint=path.name, sha256=digest,
        platform=platform.platform(), architecture=platform.machine(), torch=torch.__version__,
        torchvision=torchvision.__version__, numpy=np.__version__, device=str(device),
        actual_threads=torch.get_num_threads(), prediction_shape=list(prediction.shape),
        prototype_shape=list(extra[2].shape),
        note='Zero input and synthetic NMS boxes check interfaces only; no recognition/accuracy claim.'), indent=2))


if __name__ == '__main__':
    main()
