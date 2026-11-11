#!/usr/bin/env python3
"""Capture actual aligned D435 input and exercise the deployed GPU model, no map TF invented."""
from pathlib import Path
import argparse
import time,json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
import message_filters
ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'tests/d435_real_camera')
OUT=parser.parse_args().output.expanduser().resolve()
OUT.mkdir(parents=True,exist_ok=True)
rclpy.init()
n=Node('deployment_d435_capture')
bridge=CvBridge();data=[]
subs=[message_filters.Subscriber(n,t,p,qos_profile=qos_profile_sensor_data) for t,p in [(Image,'/camera/camera/color/image_raw'),(Image,'/camera/camera/aligned_depth_to_color/image_raw'),(CameraInfo,'/camera/camera/color/camera_info')]]
sync=message_filters.ApproximateTimeSynchronizer(subs,20,.04)
def receive(rgb,depth,info):
    if data:return
    assert rgb.header.frame_id==depth.header.frame_id==info.header.frame_id
    assert (rgb.width,rgb.height)==(depth.width,depth.height)==(info.width,info.height)
    bgr=bridge.imgmsg_to_cv2(rgb,'bgr8');z=bridge.imgmsg_to_cv2(depth,'passthrough')
    assert depth.encoding=='16UC1' and (z>0).any()
    cv2.imwrite(str(OUT/'rgb.jpg'),bgr);cv2.imwrite(str(OUT/'aligned_depth_mm.png'),z)
    d=dict(width=rgb.width,height=rgb.height,frame=info.header.frame_id,depth_encoding=depth.encoding,
           k=list(info.k),d=list(info.d),distortion_model=info.distortion_model,valid_depth_pixels=int(np.count_nonzero(z)),
           rgb_stamp=[rgb.header.stamp.sec,rgb.header.stamp.nanosec],depth_stamp=[depth.header.stamp.sec,depth.header.stamp.nanosec])
    data.append((bgr,d))
sync.registerCallback(receive)
end=time.monotonic()+25
while not data and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node();rclpy.shutdown()
if not data:raise SystemExit('No synchronized real D435 RGB-D/CameraInfo received; inspect camera log')
import torch,torchvision
from ultralytics import YOLO
assert torch.cuda.is_available()
model=YOLO(str(ROOT/'models/yolov8n-seg.pt'))
bgr,record=data[0]
for _ in range(3):result=model.predict(bgr,device=0,imgsz=640,conf=.35,retina_masks=True,verbose=False)[0]
torch.cuda.synchronize()
record.update(device=str(next(model.model.parameters()).device),classes=[model.names[int(x)] for x in result.boxes.cls.cpu().numpy()],
   boxes=result.boxes.xyxy.cpu().numpy().tolist(),confidences=result.boxes.conf.cpu().numpy().tolist(),
   mask_pixels=int((result.masks.data>0).sum().item()) if result.masks is not None else 0,
   inference_speed_ms=result.speed,note='Real live camera GPU segmentation; camera-to-odom/map TF not supplied, so no real-robot semantic map claim.')
cv2.imwrite(str(OUT/'gpu_prediction.jpg'),result.plot())
if result.masks is not None:np.savez_compressed(OUT/'gpu_masks.npz',masks=result.masks.data.cpu().numpy())
(OUT/'result.json').write_text(json.dumps(record,indent=2));print(json.dumps(record,indent=2))
