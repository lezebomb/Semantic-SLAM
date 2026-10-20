#!/usr/bin/env python3
"""Synchronize before inference and send the exact paired depth downstream."""
import copy
import json
import queue
import threading
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO
from common import COLORS, run_node


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        defaults = dict(model_path=str(Path(__file__).resolve().parents[1] / 'models/yolov8n-seg.pt'),
                        process_every_n=3, confidence=0.35, image_size=640,
                        cpu_threads=2, sync_slop=0.04, output_dir='', device='cpu', input_qos='reliable')
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if self.p['process_every_n'] < 1:
            raise ValueError('process_every_n must be positive')
        if self.p['cpu_threads'] < 1 or self.p['input_qos'] not in ('reliable', 'sensor_data'):
            raise ValueError('cpu_threads must be positive; input_qos is reliable or sensor_data')
        if self.p['device'] != 'cpu' and not torch.cuda.is_available():
            raise RuntimeError('CUDA requested but unavailable; check JetPack torch before launching')
        torch.set_num_threads(self.p['cpu_threads'])
        torch.set_num_interop_threads(1)
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.model = YOLO(self.p['model_path'])
        if self.model.task != 'segment' or len(self.model.names) != 80:
            raise ValueError('This semantic mapping format requires a COCO-80 instance segmentation checkpoint')
        # Warm up before playback to avoid dropping the first useful frames.
        self.model.predict(np.zeros((480, 640, 3), np.uint8), device=self.p['device'],
                           imgsz=self.p['image_size'], verbose=False)
        # AutoBackend/select_device resets CPU thread count during first predict.
        torch.set_num_threads(self.p['cpu_threads'])
        self.output = Path(self.p['output_dir']) if self.p['output_dir'] else None
        self.names = self.model.names
        if self.output:
            self.output.mkdir(parents=True, exist_ok=True)
            (self.output / 'class_names.json').write_text(json.dumps(self.names, indent=2))
        self.stats = dict(synchronized=0, selected=0, processed=0, overwritten=0,
                          detections=0, failures=0, inference_ms=[], classes={})
        self.next_instance = 1
        self.jobs = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.pubs = {name: self.create_publisher(Image, '/semantic/' + name, 5)
                     for name in ('image_annotated', 'label_image', 'instance_image', 'paired_depth')}
        self.info_pub = self.create_publisher(CameraInfo, '/semantic/paired_camera_info', 5)
        self.status_pub = self.create_publisher(String, '/semantic/detector_status', 5)
        reliability = (ReliabilityPolicy.BEST_EFFORT if self.p['input_qos'] == 'sensor_data'
                       else ReliabilityPolicy.RELIABLE)
        self.subs = [message_filters.Subscriber(self, typ, topic,
                                               qos_profile=QoSProfile(depth=10, reliability=reliability))
                     for typ, topic in ((Image, '/semantic/input/rgb'),
                                        (Image, '/semantic/input/depth'),
                                        (CameraInfo, '/data_throttled_camera_info'))]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.subs, queue_size=40, slop=self.p['sync_slop'])
        self.sync.registerCallback(self.receive)
        self.worker = threading.Thread(target=self.infer, daemon=True)
        self.worker.start()
        self.get_logger().info(f'YOLO_READY model={self.p["model_path"]} device={self.p["device"]} '
                               f'threads={torch.get_num_threads()} every_n={self.p["process_every_n"]}')

    def receive(self, rgb, depth, info):
        self.stats['synchronized'] += 1
        if self.stats['synchronized'] % self.p['process_every_n']:
            return
        self.stats['selected'] += 1
        if self.jobs.full():
            try:
                self.jobs.get_nowait()
                self.stats['overwritten'] += 1
            except queue.Empty:
                pass
        self.jobs.put_nowait((rgb, depth, info))

    def infer(self):
        while not self.stop.is_set():
            try:
                bundle = self.jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.process(*bundle)
            except Exception as e:
                self.stats['failures'] += 1
                self.get_logger().error(f'Inference failed: {e}')

    def process(self, rgb, depth, info):
        image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
        if (rgb.width, rgb.height) != (depth.width, depth.height):
            raise ValueError('This pipeline requires RGB-registered depth at RGB resolution')
        if depth.header.frame_id != info.header.frame_id or rgb.header.frame_id != info.header.frame_id:
            raise ValueError('RGB/depth/CameraInfo optical frames differ; registration is required')
        started = time.monotonic()
        result = self.model.predict(image, device=self.p['device'], conf=self.p['confidence'],
                                    imgsz=self.p['image_size'], retina_masks=True, verbose=False)[0]
        elapsed = (time.monotonic() - started) * 1000
        label = np.zeros(image.shape[:2], np.uint8)
        instance = np.zeros(image.shape[:2], np.uint16)
        annotated = image.copy()
        detections = []
        if result.masks is not None:
            masks = result.masks.data.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confidences = result.boxes.conf.cpu().numpy()
            # Paint low confidence first; high confidence owns overlap pixels.
            for i in np.argsort(confidences):
                mask = masks[i] > 0.5
                if mask.shape != label.shape:
                    raise ValueError('retina_masks output does not match RGB dimensions')
                cid = int(classes[i])
                iid = self.next_instance
                self.next_instance = self.next_instance % 65535 + 1
                label[mask] = cid + 1
                instance[mask] = iid
                color = COLORS[cid][::-1].astype(np.uint8)
                annotated[mask] = (0.5 * annotated[mask] + 0.5 * color).astype(np.uint8)
                box = result.boxes.xyxy[i].cpu().numpy().astype(int)
                cv2.rectangle(annotated, tuple(box[:2]), tuple(box[2:]), tuple(int(v) for v in color), 2)
                cv2.putText(annotated, f'{self.names[cid]} #{iid} {confidences[i]:.2f}',
                            (int(box[0]), max(18, int(box[1]))), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)
                name = self.names[cid]
                self.stats['classes'][name] = self.stats['classes'].get(name, 0) + 1
                detections.append(dict(class_id=cid, name=name, instance_id=iid,
                                       confidence=float(confidences[i]), pixels=int(mask.sum())))
        self.stats['processed'] += 1
        self.stats['detections'] += len(detections)
        self.stats['inference_ms'].append(round(elapsed, 2))
        # Depth stamp is the projection/TF reference; RGB differs by <= sync_slop.
        for name, array, encoding in [('label_image', label, 'mono8'),
                                       ('instance_image', instance, 'mono16'),
                                       ('image_annotated', annotated, 'bgr8')]:
            msg = self.bridge.cv2_to_imgmsg(array, encoding=encoding)
            msg.header = copy.deepcopy(depth.header)
            self.pubs[name].publish(msg)
        paired_info = copy.deepcopy(info)
        paired_info.header = copy.deepcopy(depth.header)
        self.pubs['paired_depth'].publish(depth)
        self.info_pub.publish(paired_info)
        rgb_ns = rgb.header.stamp.sec * 10**9 + rgb.header.stamp.nanosec
        depth_ns = depth.header.stamp.sec * 10**9 + depth.header.stamp.nanosec
        status = dict(frame=self.stats['processed'], inference_ms=round(elapsed, 2),
                      capture_stamp_ns=depth_ns, rgb_depth_delta_ms=(rgb_ns-depth_ns)/1e6,
                      detections=detections)
        self.status_pub.publish(String(data=json.dumps(status)))
        if self.output and detections and (self.stats['processed'] == 1 or self.stats['processed'] % 25 == 0):
            cv2.imwrite(str(self.output / 'latest_annotated.jpg'), annotated)
            cv2.imwrite(str(self.output / 'latest_class_mask.png'), label)
            cv2.imwrite(str(self.output / 'latest_instance_mask.png'), instance)
            (self.output / 'latest_detection.json').write_text(json.dumps(status, indent=2))
        if self.stats['processed'] % 25 == 1:
            self.get_logger().info(f"frame={self.stats['processed']} objects={len(detections)} inference={elapsed:.0f}ms")

    def close(self):
        self.stop.set()
        self.worker.join(timeout=10)
        if self.output:
            (self.output / 'detector_stats.json').write_text(json.dumps(self.stats, indent=2))


if __name__ == '__main__':
    run_node(YoloDetector)
