#!/usr/bin/env python3
"""Standard held-out val(), followed by real mask overlays on unseen images."""
import argparse
import gc
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent


def trained_weight():
    proof_path, path = ROOT / 'results/training_summary.json', ROOT / 'weights/best.pt'
    if not proof_path.exists() or not path.exists():
        raise SystemExit('Formal trained best.pt and gradient evidence are absent; refusing final evaluation.')
    proof = json.loads(proof_path.read_text())
    assert proof['formal_training_completed'] and proof['optimizer_updates'] > 0
    assert proof['best_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert proof['projection_max_weight_change'] > 0
    return path


def metrics_dict(metrics):
    return {prefix + suffix: float(getattr(part, attribute))
            for prefix, part in [('box_', metrics.box), ('mask_', metrics.seg)]
            for suffix, attribute in [('precision', 'mp'), ('recall', 'mr'),
                                      ('map50', 'map50'), ('map50_95', 'map')]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='0' if torch.cuda.is_available() else 'cpu')
    args = parser.parse_args()
    best = trained_weight()
    manifest = json.loads((ROOT / 'data/dataset/manifest.json').read_text())
    assert (ROOT / 'data/dataset/images_complete.json').exists()
    results = dict(device=args.device, imgsz=640, fp32=True, split='test', test_images=manifest['sizes']['test'],
                   selection_validation_is_separate=True, training_conditions_not_equal=True,
                   supported_test_classes=sum(v > 0 for v in manifest['class_instance_counts']['test'].values()),
                   models={})
    for kind, path in [('baseline', ROOT / 'weights/yolov8n-seg.pt'), ('mobilenet', best)]:
        model = YOLO(str(path), task='segment', verbose=False)
        metrics = model.val(data=str(ROOT / 'data/coco_subset.yaml'), split='test', imgsz=640,
                            batch=8, device=args.device, workers=2, half=False, plots=True,
                            conf=0.001, iou=0.7, max_det=300,
                            project=str(ROOT / 'evaluation'), name=kind, exist_ok=False)
        results['models'][kind] = metrics_dict(metrics)
        (ROOT / 'results/accuracy.json').write_text(json.dumps(results, indent=2) + '\n')
        del model, metrics
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    (ROOT / 'results/accuracy.json').write_text(json.dumps(results, indent=2) + '\n')
    indoor = sorted((ROOT / 'images').glob('*.jpg'))
    held_out = [ROOT / 'data/dataset/images' / row['official_split'] / row['file']
                for row in manifest['records']['test'][:6]]
    samples = [('indoor', path) for path in indoor] + [('coco_test', path) for path in held_out]
    output = ROOT / 'results/comparison_images'
    output.mkdir(exist_ok=True)
    predictions = dict(baseline=[], mobilenet=[])
    for kind, path in [('baseline', ROOT / 'weights/yolov8n-seg.pt'), ('mobilenet', best)]:
        model = YOLO(str(path), task='segment', verbose=False)
        for index, (source, image_path) in enumerate(samples):
            result = model.predict(str(image_path), device=args.device, imgsz=640, rect=False,
                                   conf=0.25, iou=0.7, max_det=100, retina_masks=True,
                                   half=False, verbose=False)[0]
            annotations = []
            for i, box in enumerate(result.boxes):
                pixels = int((result.masks.data[i] > 0.5).sum()) if result.masks is not None else 0
                annotations.append(dict(class_id=int(box.cls), class_name=result.names[int(box.cls)],
                    confidence=float(box.conf), xyxy=[float(x) for x in box.xyxy[0]], mask_pixels=pixels))
            predictions[kind].append(dict(image=str(image_path), source=source, detections=annotations))
            overlay = result.plot()
            # The picture contains only model-generated boxes and masks.
            cv2.imwrite(str(output / f'{index:02d}_{kind}.jpg'), overlay)
        del model
        gc.collect()
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
    for index in range(len(samples)):
        panels = [cv2.imread(str(output / f'{index:02d}_{kind}.jpg')) for kind in ['baseline', 'mobilenet']]
        height = min(image.shape[0] for image in panels)
        panels = [cv2.resize(image, (round(image.shape[1] * height / image.shape[0]), height)) for image in panels]
        for position, label in enumerate(['YOLOv8n-seg', 'MobileNetV3-YOLOv8n-seg (trained)']):
            panels[position] = cv2.copyMakeBorder(panels[position], 32, 0, 0, 0,
                                                 cv2.BORDER_CONSTANT, value=(25, 25, 25))
            cv2.putText(panels[position], label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(output / f'{index:02d}_side_by_side.jpg'), np.hstack(panels))
    indoor_detected = sum(any(d['confidence'] >= 0.35 and d['mask_pixels'] >= 16 for d in item['detections'])
                         for item in predictions['mobilenet'] if item['source'] == 'indoor')
    predictions['rtabmap_gate'] = dict(
        indoor_images_with_valid_masks=indoor_detected,
        independent_mask_map50=results['models']['mobilenet']['mask_map50'],
        eligible=indoor_detected > 0 and results['models']['mobilenet']['mask_map50'] > 0.01,
        note='This is a minimal execution gate, not a claim that every detected object is correct.')
    (ROOT / 'results/predictions.json').write_text(json.dumps(predictions, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    print('RTAB-Map eligibility:', predictions['rtabmap_gate'])


if __name__ == '__main__':
    main()
