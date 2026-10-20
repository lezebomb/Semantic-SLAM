#!/usr/bin/env python3
"""Strict ImageNet backbone and semantically remapped YOLO neck/head initialization."""
import hashlib
import json
import os
from pathlib import Path
import torch
from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent


def initialize():
    torch.manual_seed(20261015)
    torch.set_num_threads(2)
    ROOT.joinpath('weights').mkdir(exist_ok=True)
    ROOT.joinpath('results').mkdir(exist_ok=True)
    target = YOLO(str(ROOT / 'yolov8n-mobilenetv3-small-seg.yaml'), task='segment', verbose=False)
    source = YOLO(str(ROOT / 'weights/yolov8n-seg.pt'), task='segment', verbose=False)
    pretrained = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.IMAGENET1K_V1)
    backbone_state = pretrained.features[:-1].state_dict()
    target.model.model[0].m.load_state_dict(backbone_state, strict=True)
    for key, tensor in backbone_state.items():
        assert torch.equal(tensor, target.model.model[0].m.state_dict()[key])
    mapping = {9: 7, **{i: i - 2 for i in range(10, 23)}}
    migrated = []
    for src, dst in mapping.items():
        a, b = source.model.model[src], target.model.model[dst]
        assert type(a) is type(b), (src, dst)
        assert list(a.state_dict()) == list(b.state_dict()), (src, dst)
        for key in a.state_dict():
            assert a.state_dict()[key].shape == b.state_dict()[key].shape, (src, dst, key)
        if src >= 10:
            f = a.f if isinstance(a.f, list) else [a.f]
            routing = {4: 2, 6: 4, **mapping}
            expected = [routing[i] if i >= 0 else i for i in f]
            assert expected == (b.f if isinstance(b.f, list) else [b.f])
        b.load_state_dict(a.state_dict(), strict=True)
        assert all(torch.equal(v, b.state_dict()[k]) for k, v in a.state_dict().items())
        migrated.append(dict(source_layer=src, target_layer=dst, type=type(a).__name__,
                             parameters=sum(p.numel() for p in b.parameters())))
    target.model.names = dict(source.model.names)
    target.model.args = dict(source.model.args)
    target.ckpt = {'epoch': -1, 'training_stage': 'initialized_untrained',
                   'train_args': dict(source.model.args), 'gradient_updates': 0}
    path = ROOT / 'weights/initialized_untrained.pt'
    target.save(path)
    loaded = YOLO(str(path), task='segment', verbose=False)
    assert loaded.model.names == source.model.names
    # Explicitly exercise the stock trainer's reconstruction/load route.
    from ultralytics.models.yolo.segment.train import SegmentationTrainer
    trainer = object.__new__(SegmentationTrainer)
    trainer.data = {'nc': 80, 'channels': 3}
    reconstructed = trainer.get_model(cfg=loaded.model.yaml, weights=loaded.model, verbose=False)
    assert all(torch.equal(v, reconstructed.state_dict()[k]) for k, v in loaded.model.state_dict().items())
    reconstructed.eval()
    with torch.inference_mode():
        output = reconstructed(torch.zeros(1, 3, 640, 640))
    assert list(output[0].shape) == [1, 116, 8400]
    assert list(output[1][2].shape) == [1, 32, 160, 160]
    total = sum(p.numel() for p in target.model.parameters())
    mobile_params = sum(p.numel() for p in target.model.model[0].parameters())
    yolo_params = sum(row['parameters'] for row in migrated)
    report = dict(status='initialized_untrained', real_formal_training_completed=False,
                  architecture_parameters=total, imagenet_backbone_parameters=mobile_params,
                  transferred_yolo_parameters=yolo_params,
                  random_projection_parameters=total - mobile_params - yolo_params,
                  imagenet_source=MobileNet_V3_Small_Weights.IMAGENET1K_V1.url,
                  baseline_sha256=hashlib.sha256((ROOT / 'weights/yolov8n-seg.pt').read_bytes()).hexdigest(),
                  initialized_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  strict_state_dict_and_routing_checks_passed=True,
                  stock_trainer_reconstruction_preserves_initialization=True,
                  yolo_layer_mapping=migrated)
    (ROOT / 'results/initialization.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return path


if __name__ == '__main__':
    initialize()
