#!/usr/bin/env python3
"""Record actual synchronized RGB/predictions and live map messages, without GUI."""
from collections import OrderedDict
import json
import math
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from rtabmap_msgs.msg import MapGraph
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'semantic'))
from common import POINT_DTYPE

FPS=3


def stamp(msg): return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


def title(image, text, color=(255,255,255), y=0):
    cv2.rectangle(image,(0,y),(image.shape[1],y+28),(25,25,25),-1)
    cv2.putText(image,text,(8,y+20),cv2.FONT_HERSHEY_SIMPLEX,.47,color,1,cv2.LINE_AA)


def fit(image):
    h,w=image.shape[:2];scale=min(640/w,480/h)
    resized=cv2.resize(image,(max(1,round(w*scale)),max(1,round(h*scale))),interpolation=cv2.INTER_NEAREST)
    canvas=np.full((480,640,3),45,np.uint8)
    y=(480-resized.shape[0])//2;x=(640-resized.shape[1])//2
    canvas[y:y+resized.shape[0],x:x+resized.shape[1]]=resized
    return canvas


class Video(Node):
    def __init__(self, output):
        super().__init__('handpush_video_recorder')
        cv2.setNumThreads(1)
        self.output=output;self.bridge=CvBridge();self.cache=OrderedDict()
        self.original=None;self.annotated=None;self.capture_ns=0
        self.grid=None;self.graph=[];self.points=None
        self.start_wall=None;self.frames=0;self.frame_times=[]
        self.counts=dict(rgb=0,annotated=0,unpaired_annotated=0,map=0,graph=0,semantic=0)
        self.writers={}
        for name in ['camera_segmentation','mapping_process']:
            writer=cv2.VideoWriter(str(output/(name+'.mp4')),cv2.VideoWriter_fourcc(*'mp4v'),FPS,(640,960))
            if not writer.isOpened(): raise RuntimeError('MPEG-4 video encoder unavailable: '+name)
            self.writers[name]=writer
        self.subs=[
            self.create_subscription(Image,'/camera/camera/color/image_raw',self.rgb,qos_profile_sensor_data),
            self.create_subscription(Image,'/semantic/image_annotated',self.prediction,qos_profile_sensor_data),
            self.create_subscription(OccupancyGrid,'/map',self.map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(MapGraph,'/mapGraph',self.path,qos_profile_sensor_data),
            self.create_subscription(PointCloud2,'/semantic/map',self.semantic,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
        self.create_timer(1/FPS,self.render)

    def rgb(self,msg):
        self.counts['rgb']+=1
        self.cache[stamp(msg)]=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        while len(self.cache)>90:self.cache.popitem(last=False)

    def prediction(self,msg):
        self.counts['annotated']+=1
        raw=self.cache.get(stamp(msg))
        if raw is None:self.counts['unpaired_annotated']+=1;return
        self.original=raw.copy();self.annotated=self.bridge.imgmsg_to_cv2(msg,'bgr8')
        self.capture_ns=stamp(msg)

    def map(self,msg):self.grid=msg;self.counts['map']+=1

    def path(self,msg):
        self.graph=[(p.position.x,p.position.y) for p in msg.poses]
        self.counts['graph']+=1

    def semantic(self,msg):
        self.points=np.frombuffer(msg.data,dtype=POINT_DTYPE).copy()
        self.counts['semantic']+=1

    def xy_pixels(self,x,y):
        info=self.grid.info;q=info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        dx=np.asarray(x)-info.origin.position.x;dy=np.asarray(y)-info.origin.position.y
        u=(math.cos(yaw)*dx+math.sin(yaw)*dy)/info.resolution
        v=info.height-1-(-math.sin(yaw)*dx+math.cos(yaw)*dy)/info.resolution
        return np.column_stack([u,v]).astype(np.int32)

    def maps(self):
        if self.grid is None:
            canvas=np.full((960,640,3),45,np.uint8)
            title(canvas,'Waiting for actual map messages')
            return canvas
        info=self.grid.info;data=np.asarray(self.grid.data).reshape(info.height,info.width)
        image=np.full(data.shape,205,np.uint8);image[data==0]=254
        known=data>0;image[known]=np.rint(254*(1-data[known]/100)).astype(np.uint8)
        geo=cv2.cvtColor(np.flipud(image),cv2.COLOR_GRAY2BGR)
        sem=np.full_like(geo,245)
        if self.points is not None and len(self.points):
            pts=self.points;xy=self.xy_pixels(pts['x'],pts['y']);rgb=pts['rgb'].view(np.uint32)
            ok=(xy[:,0]>=0)&(xy[:,0]<info.width)&(xy[:,1]>=0)&(xy[:,1]<info.height)
            colors=np.column_stack([rgb&255,(rgb>>8)&255,(rgb>>16)&255]).astype(np.uint8)
            sem[xy[ok,1],xy[ok,0]]=colors[ok]
        if self.graph:
            path=np.asarray(self.graph);xy=self.xy_pixels(path[:,0],path[:,1]).reshape(-1,1,2)
            for panel in [geo,sem]:
                cv2.polylines(panel,[xy],False,(0,0,255),1)
                cv2.circle(panel,tuple(xy[0,0]),2,(0,180,0),-1)
                cv2.circle(panel,tuple(xy[-1,0]),2,(0,0,255),-1)
        geo=fit(geo);sem=fit(sem)
        title(geo,f'Geometry map + graph | poses={len(self.graph)} | update={self.counts["map"]}')
        title(sem,f'Actual semantic voxels, XY view | points={len(self.points) if self.points is not None else 0}')
        return np.vstack([geo,sem])

    def render(self):
        if self.annotated is None:return
        now=time.time()
        if self.start_wall is None:self.start_wall=now
        elapsed=now-self.start_wall
        camera=np.vstack([self.original.copy(),self.annotated.copy()])
        title(camera,f'Original RGB | elapsed={elapsed:.1f}s')
        title(camera,'Original YOLOv8n-seg CUDA predictions',y=480)
        maps=self.maps()
        try:state=json.loads((self.output/'live_status.json').read_text())
        except (OSError,ValueError):state={}
        resource=state.get('resource_latest',{})
        cpu=resource.get('cpu_percent_sum',0);peak=state.get('cpu_percent_sum_peak',0)
        inference=(state.get('detector') or {}).get('inference_ms',0)
        tracking='VALID' if state.get('tracking_valid') else 'WAIT / LOST'
        color=(0,220,0) if state.get('tracking_valid') else (0,0,255)
        title(camera,f'YOLO predict={inference:.1f} ms | CPU={cpu:.0f}% peak={peak:.0f}% (1 core=100%)',y=932)
        title(maps,f'Visual tracking: {tracking} | elapsed={elapsed:.1f}s',color,y=932)
        expected=int(elapsed*FPS)+1
        while self.frames<expected:
            self.writers['camera_segmentation'].write(camera)
            self.writers['mapping_process'].write(maps)
            self.frame_times.append(now);self.frames+=1
        self.save_metrics()

    def save_metrics(self):
        (self.output/'video_metrics.json').write_text(json.dumps(dict(
            fps=FPS,frames=self.frames,encoded_duration_seconds=self.frames/FPS,
            start_wall_time=self.start_wall,capture_stamp_ns=self.capture_ns,
            latest_render_wall_time=time.time(),received=self.counts,
            layout='Camera video: original RGB above synchronized real masks; map video: actual geometry+graph above actual semantic XY projection',
            repeated_frames='Latest actual messages held until a newer message; no interpolated maps or fabricated predictions'),indent=2))

    def close(self):
        for writer in self.writers.values():writer.release()
        self.save_metrics()


if __name__=='__main__':
    rclpy.init();node=Video(Path(sys.argv[1]))
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
