#!/usr/bin/env python3
"""Exercise ROS native modules, CUDA NMS and real YOLO instance masks on the Jetson."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'tests' / ('gpu-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))


def main():
    OUT.mkdir(exist_ok=False)
    report = {'success': False, 'device': 'cuda:0', 'images': [], 'errors': []}
    monitor = None
    log = (OUT / 'tegrastats.log').open('w')
    try:
        import cv2
        import cv_bridge
        import numpy as np
        import rclpy
        import tf2_ros
        import torch
        import torchvision
        from ultralytics import YOLO
        assert tuple(__import__('sys').version_info[:2]) == (3, 10)
        assert int(np.__version__.split('.')[0]) == 1
        bridge = cv_bridge.CvBridge()
        for encoding, sample in [('bgr8', np.zeros((4, 4, 3), np.uint8)),
                                 ('32FC1', np.ones((4, 4), np.float32)),
                                 ('16UC1', np.ones((4, 4), np.uint16))]:
            assert np.array_equal(bridge.imgmsg_to_cv2(bridge.cv2_to_imgmsg(sample, encoding), encoding), sample)
        assert torch.cuda.is_available(), 'CUDA unavailable; no CPU fallback accepted'
        torch.set_num_threads(2)
        torch.set_num_interop_threads(1)
        cv2.setNumThreads(1)
        boxes = torch.tensor([[0., 0., 10., 10.], [1., 1., 9., 9.]], device='cuda')
        result = torchvision.ops.nms(boxes, torch.tensor([.9, .8], device='cuda'), .5)
        torch.cuda.synchronize()
        assert result.tolist() == [0], 'CUDA NMS failed'
        path = ROOT / 'models/yolov8n-seg.pt'
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == 'a7cd8f929e1903d78a12a48efecab430209f18dc46cb96c3599a5980c63c423c'
        model = YOLO(str(path), task='segment')
        assert model.task == 'segment' and len(model.names) == 80
        images = sorted((ROOT / 'tests/images').glob('rgb_*.jpg'))
        assert len(images) == 20
        try:
            monitor = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=log, stderr=subprocess.STDOUT)
        except FileNotFoundError:
            report['telemetry_unavailable'] = 'tegrastats missing'
        def predict(img):
            return model.predict(img, device=0, conf=.35, imgsz=640, retina_masks=True, verbose=False)[0]
        first = cv2.imread(str(images[0]))
        for _ in range(5):
            predict(first)
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
        elapsed = []
        start_cpu = time.process_time()
        start_wall = time.perf_counter()
        for i, image in enumerate(images):
            rgb = cv2.imread(str(image))
            assert rgb is not None
            torch.cuda.synchronize()
            before = time.perf_counter()
            pred = predict(rgb)
            torch.cuda.synchronize()
            classes = pred.boxes.cls.cpu().numpy().astype(int)
            confidences = pred.boxes.conf.cpu().numpy()
            xyxy = pred.boxes.xyxy.cpu().numpy()
            masks = pred.masks.data.cpu().numpy() if pred.masks is not None else np.empty((0, *rgb.shape[:2]))
            ms = (time.perf_counter() - before) * 1000
            elapsed.append(ms)
            assert len(classes) == len(masks)
            assert np.isfinite(xyxy).all()
            if len(masks):
                assert masks.shape[1:] == rgb.shape[:2]
            record = dict(image=image.name, elapsed_ms=ms, detections=len(classes),
                mask_pixels=int(np.count_nonzero(masks)), boxes=xyxy.tolist(),
                classes=[model.names[int(c)] for c in classes], confidences=confidences.tolist(),
                ultralytics_speed_ms=pred.speed)
            report['images'].append(record)
            if len(masks) and sum(x['mask_pixels'] > 0 for x in report['images']) <= 3:
                assert cv2.imwrite(str(OUT / f'{image.stem}_prediction.jpg'), pred.plot())
                np.savez_compressed(OUT / f'{image.stem}_masks.npz', masks=masks, classes=classes, boxes=xyxy)
        wall = time.perf_counter() - start_wall
        report.update(torch=str(torch.__version__), torchvision=str(torchvision.__version__),
            cuda_runtime=torch.version.cuda, cudnn=torch.backends.cudnn.version(),
            gpu=torch.cuda.get_device_name(0), numpy=np.__version__, cv2=cv2.__version__,
            cv_bridge=cv_bridge.__file__, rclpy=rclpy.__file__, sha256=digest,
            cuda_nms='passed', ros_native_conversions='passed', frames=len(elapsed),
            p50_ms=float(np.percentile(elapsed, 50)), p95_ms=float(np.percentile(elapsed, 95)),
            fps=1000/float(np.mean(elapsed)), cpu_percent_one_core=100*(time.process_time()-start_cpu)/wall,
            peak_rss_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
            gpu_peak_allocated_MiB=torch.cuda.max_memory_allocated()/1024**2,
            gpu_peak_reserved_MiB=torch.cuda.max_memory_reserved()/1024**2,
            detections=sum(x['detections'] for x in report['images']),
            frames_with_masks=sum(x['mask_pixels'] > 0 for x in report['images']),
            actual_threads=torch.get_num_threads(),
            precision='FP32', warmup_frames=5,
            timing='CUDA synchronized predict + transfer boxes/masks to CPU; image reading excluded')
        assert report['frames_with_masks'] > 0, 'No nonempty masks on real images'
        report['success'] = True
    except Exception:
        report['errors'].append(traceback.format_exc())
    finally:
        if monitor is not None:
            monitor.send_signal(signal.SIGTERM)
            try: monitor.wait(timeout=5)
            except subprocess.TimeoutExpired:
                monitor.kill(); monitor.wait()
        log.close()
        (OUT / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k != 'images'}, ensure_ascii=False, indent=2))
        print('Output:', OUT)
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
