"""Physical TF isolation, capture-time checks, actual video and map snapshots."""
import copy
import csv
import json
import math
import threading
import time
from collections import Counter, deque
from pathlib import Path
import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rtabmap_msgs.msg import Info, MapGraph
import tf2_ros
from mapping_config import tf_topics

SENSOR = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
def stamp_ns(header): return header.stamp.sec*10**9+header.stamp.nanosec

def atomic_json(path, value):
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2));tmp.replace(path)

def image_bgr(msg):
    if msg.encoding not in ('rgb8','bgr8'): raise ValueError('Expected RGB/BGR image, got '+msg.encoding)
    a = np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
    return a[:,:,::-1].copy() if msg.encoding == 'rgb8' else a.copy()

def write_cloud(path, msg):
    fields = {f.name:f for f in msg.fields}
    if not all(k in fields for k in ('x','y','z')): return 0
    endian = '>' if msg.is_bigendian else '<'
    xyz = []
    data = bytes(msg.data)
    for name in ('x','y','z'):
        field = fields[name]
        dtype = endian+('f8' if field.datatype == 8 else 'f4')
        xyz.append(np.ndarray((msg.height,msg.width),dtype=dtype,buffer=data,
                   offset=field.offset,strides=(msg.row_step,msg.point_step)).reshape(-1))
    points = np.column_stack(xyz); valid = np.all(np.isfinite(points),axis=1)
    colors = np.full((len(points),3),190,dtype=np.uint8)
    if 'rgb' in fields or 'rgba' in fields:
        f = fields.get('rgb',fields.get('rgba'))
        bits = np.ndarray((msg.height,msg.width),dtype=endian+'u4',buffer=data,
                          offset=f.offset,strides=(msg.row_step,msg.point_step)).reshape(-1)
        colors = np.column_stack(((bits>>16)&255,(bits>>8)&255,bits&255)).astype('u1')
    points,colors = points[valid],colors[valid]
    dtype = np.dtype([(k,'<f4') for k in ('x','y','z')]+[(k,'u1') for k in ('red','green','blue')])
    result = np.empty(len(points),dtype=dtype)
    for i,k in enumerate(('x','y','z')):result[k]=points[:,i]
    for i,k in enumerate(('red','green','blue')):result[k]=colors[:,i]
    header = 'ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(points))+'\n'
    header += '\n'.join('property float '+k for k in ('x','y','z'))+'\n'
    header += '\n'.join('property uchar '+k for k in ('red','green','blue'))+'\nend_header\n'
    with path.open('wb') as f:f.write(header.encode());f.write(result.tobytes())
    # Genuine XY projection; no interpolation or invented walls.
    if len(points):
        lo=points[:,:2].min(axis=0);hi=points[:,:2].max(axis=0);span=np.maximum(hi-lo,.1)
        xy=(points[:,:2]-lo)/max(span)*950+25;xy=xy.astype(int)
        canvas=np.full((1000,1000,3),245,np.uint8)
        canvas[999-xy[:,1],xy[:,0]]=colors[:,::-1]
        cv2.imwrite(str(path.with_suffix('.png')),canvas)
    return len(points)

class Runtime(Node):
    def __init__(self, config, output, sim=False, video=True):
        super().__init__('runtime',namespace=config['namespace'],parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=sim)])
        self.c=config;self.output=output;self.started=time.monotonic();self.subs=[]
        self.counts=Counter();self.arrivals={};self.errors=[];self.fatal=''
        self.sync_count=0;self.sync_spreads=[];self.tf_ok=0;self.tf_failed=0;self.pending=deque(maxlen=40)
        self.graph_poses=0;self.loop_closures=0;self.geometry=None;self.occupancy=None
        self.detector={};self.grid={};self.private_edges=Counter();self.invalid_odom=0
        self.initial_verified=False
        self.images={};self.video_enabled=video;self.video_writer=None;self.video_frames=0
        self.video_start=None;self.video_tick=0;self.video_path=None;self.last_tf_error=''
        self.video_stop=threading.Event();self.video_thread=None;self.video_skipped_ticks=0
        self.video_sample_handle=None
        self.static={};self.tf_buffer=tf2_ros.Buffer(cache_time=Duration(seconds=60))
        tf_topic, static_topic=tf_topics(config)
        # Receive TF independently from image encoding and diagnostic lookups.
        # Keep the same private topics, timestamps and readiness conditions.
        from rclpy.executors import SingleThreadedExecutor
        self.tf_node=Node('capture_tf_receiver',namespace=config['namespace'],
            cli_args=['--ros-args','-r','/tf:='+tf_topic,'-r','/tf_static:='+static_topic],
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=sim)])
        self.tf_listener=tf2_ros.TransformListener(self.tf_buffer,self.tf_node,spin_thread=False)
        self.tf_executor=SingleThreadedExecutor();self.tf_executor.add_node(self.tf_node)
        self.tf_thread=threading.Thread(target=self.tf_executor.spin,name='capture-tf-receiver',daemon=True)
        self.tf_thread.start()
        self.tf_pub=self.create_publisher(TFMessage,tf_topic,30)
        self.static_pub=self.create_publisher(TFMessage,static_topic,LATCHED)
        self.add(TFMessage,tf_topic,self.dynamic_tf,30)
        self.add(TFMessage,'/tf_static',self.camera_static,LATCHED)
        for m in config['mounts']:
            tr=TransformStamped();tr.header.frame_id=m['parent'];tr.child_frame_id=m['child']
            tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,m['translation'])
            q=tr.transform.rotation;q.x,q.y,q.z,q.w=map(float,m['rotation_xyzw']);self.static[tr.child_frame_id]=tr
        self.publish_static()
        self.filters=[message_filters.SimpleFilter() for _ in range(3)]
        self.sync=message_filters.ApproximateTimeSynchronizer(self.filters,40,config['sync_seconds'])
        self.sync.registerCallback(self.matched)
        for i,(key,typ) in enumerate([('rgb',Image),('depth',Image),('camera_info',CameraInfo)]):
            self.add(typ,config['topics'][key],lambda m,k=key,n=i:self.camera(k,n,m),SENSOR)
        self.add(Odometry,config['topics']['odom'],self.odom,SENSOR)
        ns=config['namespace']
        self.add(Info,ns+'/info',self.info,10)
        self.add(MapGraph,ns+'/mapGraph',self.graph,LATCHED)
        self.add(PointCloud2,ns+'/cloud_map',self.cloud,LATCHED)
        self.add(OccupancyGrid,ns+'/map',self.map,LATCHED)
        self.add(String,'/semantic/detector_status',self.detector_status,10)
        self.add(String,'/semantic/map_status',self.map_status,10)
        self.add(PointCloud2,'/semantic/points_camera',self.camera_points,10)
        self.add(PointCloud2,'/semantic/pointcloud',lambda m:self.mark('points_map'),10)
        self.add(PointCloud2,'/semantic/map',lambda m:self.mark('semantic_map'),LATCHED)
        self.add(Image,'/semantic/image_annotated',self.annotated,5)
        self.save_client=self.create_client(Trigger,'/semantic/save_map')
        self.timeline=(output/'timeline.csv').open('w',newline='');self.csv=csv.writer(self.timeline)
        self.csv.writerow(['wall_time','ros_time_ns','rgb','depth','sync','rtabmap_info','graph_poses',
                          'geometry_points','tf_ok','tf_failed','points_camera','points_map','voxels'])

    def add(self,typ,topic,cb,qos):self.subs.append(self.create_subscription(typ,topic,cb,qos))
    def mark(self,key):self.counts[key]+=1;self.arrivals[key]=time.monotonic()
    def publish_static(self):
        for tr in self.static.values():tr.header.stamp=self.get_clock().now().to_msg()
        self.static_pub.publish(TFMessage(transforms=list(self.static.values())))
    def camera_static(self,msg):
        changed=False
        for tr in msg.transforms:
            # Only camera-internal rigid links are imported. Never import map/odom.
            p,ch=tr.header.frame_id,tr.child_frame_id
            if p.startswith('camera_') and ch.startswith('camera_') and p!=ch and ch!=self.c['frames']['camera']:
                self.static[ch]=copy.deepcopy(tr);changed=True
        if changed:self.publish_static()
    def private_static(self,msg):
        for tr in msg.transforms:self.tf_buffer.set_transform_static(tr,'owned_static_bridge')
    def dynamic_tf(self,msg):
        for tr in msg.transforms:
            self.private_edges[tr.header.frame_id+' -> '+tr.child_frame_id]+=1
            if tr.header.frame_id==self.c['frames']['map'] and tr.child_frame_id==self.c['frames']['odom']:
                self.mark('map_tf')
    def camera(self,key,index,msg):
        self.mark(key);self.filters[index].signalMessage(msg)
        if key=='rgb' and self.video_enabled:
            try:self.images['rgb']=(stamp_ns(msg.header),image_bgr(msg))
            except ValueError as e:self.fatal=str(e)
        if key=='depth':
            if msg.encoding not in ('16UC1','32FC1'):self.fatal='Unsupported depth encoding: '+msg.encoding
        if key=='camera_info' and (msg.k[0]<=0 or msg.k[4]<=0):self.fatal='Invalid camera intrinsics'
    def matched(self,rgb,depth,info):
        self.sync_count+=1
        if self.sync_count%3==0:
            self.pending.append((time.monotonic(),copy.deepcopy(depth.header)))
        stamps=[stamp_ns(m.header) for m in (rgb,depth,info)]
        self.sync_spreads.append((max(stamps)-min(stamps))/1e6)
        if len(self.sync_spreads)>5000:self.sync_spreads=self.sync_spreads[-5000:]
    def odom(self,msg):
        self.mark('odom');p=msg.pose.pose.position;q=msg.pose.pose.orientation
        vals=[p.x,p.y,p.z,q.x,q.y,q.z,q.w];norm=sum(v*v for v in vals[3:])
        if (msg.header.frame_id!=self.c['frames']['odom'] or msg.child_frame_id!=self.c['frames']['base']
            or not all(math.isfinite(v) for v in vals) or abs(norm-1)>.01 or msg.pose.covariance[0]>=999):
            self.invalid_odom+=1;return
        # RTAB-Map rgbd_odometry owns odom -> camera_link TF. Observe only.
        self.mark('valid_odom')
    def info(self,msg):
        self.mark('rtabmap_info');self.loop_closures+=int(msg.loop_closure_id>0)
    def graph(self,msg):self.mark('graph');self.graph_poses=len(msg.poses)
    def cloud(self,msg):self.mark('geometry');self.geometry=msg
    def map(self,msg):self.mark('occupancy');self.occupancy=msg
    def detector_status(self,msg):self.mark('detector');self.detector=json.loads(msg.data)
    def map_status(self,msg):self.mark('grid');self.grid=json.loads(msg.data)
    def camera_points(self,msg):
        self.mark('points_camera');self.pending.append((time.monotonic(),copy.deepcopy(msg.header)))
    def annotated(self,msg):
        self.mark('annotated')
        if self.video_enabled:
            try:self.images['annotated']=(stamp_ns(msg.header),image_bgr(msg))
            except ValueError as e:self.fatal=str(e)
    def tick(self):
        for _ in range(len(self.pending)):
            began,header=self.pending.popleft()
            try:
                self.tf_buffer.lookup_transform(self.c['frames']['map'],header.frame_id,Time.from_msg(header.stamp))
                self.tf_ok+=1
                self.mark('capture_tf')
            except tf2_ros.TransformException as e:
                self.last_tf_error=str(e)
                if time.monotonic()-began<5:self.pending.append((began,header))
                else:self.tf_failed+=1
        self.video_tick_once()
    def video_tick_once(self):
        if not self.video_enabled or 'rgb' not in self.images:return
        if self.video_thread is None:
            # Encoding must not block ROS callbacks or capture-time TF checks.
            # The writer owns its encoder; callbacks only replace actual frames.
            self.video_thread=threading.Thread(target=self.video_loop,name='mapping-video',daemon=True)
            self.video_thread.start()
    def video_loop(self):
        fps=self.c['video']['fps'];period=1./fps;next_tick=time.monotonic()
        self.video_start=next_tick
        try:
            self.video_sample_handle=(self.output/'video_frames.csv').open('w',newline='')
            samples=csv.writer(self.video_sample_handle)
            samples.writerow(['frame','encoded_wall_time','rgb_capture_ns','annotation_capture_ns','skipped_schedule_ticks'])
            while not self.video_stop.is_set():
                wait=next_tick-time.monotonic()
                if wait>0 and self.video_stop.wait(wait):break
                stamp,rgb=self.images['rgb'];h,w=rgb.shape[:2]
                if self.video_writer is None:
                    self.video_path=self.output/'camera_segmentation.mp4'
                    self.video_writer=cv2.VideoWriter(str(self.video_path),cv2.VideoWriter_fourcc(*'mp4v'),fps,(w,h*2))
                    if not self.video_writer.isOpened():raise RuntimeError('Cannot open video encoder')
                image=np.zeros((h*2,w,3),np.uint8);image[:h]=rgb
                cv2.putText(image,'RGB capture_ns='+str(stamp),(8,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                annotation=self.images.get('annotated');annotation_stamp=None
                if annotation:
                    annotation_stamp,a=annotation;image[h:]=cv2.resize(a,(w,h))
                    cv2.putText(image,'Actual YOLO capture_ns='+str(annotation_stamp),(8,h+20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
                else:cv2.putText(image,'Waiting for actual YOLO output',(8,h+30),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
                self.video_writer.write(image);self.video_frames+=1
                samples.writerow([self.video_frames,time.time(),stamp,annotation_stamp,self.video_skipped_ticks])
                if self.video_frames%10==0:self.video_sample_handle.flush()
                next_tick+=period
                # Never replay an unbounded catch-up loop inside the TF executor.
                # Report missed encoding ticks, rather than conceal them as FPS.
                now=time.monotonic()
                if now-next_tick>=period:
                    missed=int((now-next_tick)/period);self.video_skipped_ticks+=missed;next_tick+=missed*period
        except Exception as e:self.fatal='Video encoding failed: '+repr(e)
        finally:
            if self.video_writer:self.video_writer.release()
            if self.video_sample_handle:self.video_sample_handle.close()
    def readiness(self, initial=True):
        fresh=lambda k:self.counts[k]>0 and time.monotonic()-self.arrivals[k]<self.c['runtime']['stream_timeout']
        required=['rgb','depth','camera_info','valid_odom','rtabmap_info','detector','map_tf','capture_tf']
        if initial:required+=['points_camera','points_map']
        missing=[k for k in required if not fresh(k)]
        if self.sync_count<3:missing.append('RGB/Depth/CameraInfo matching')
        if self.graph_poses<1:missing.append('RTAB-Map graph')
        if self.geometry is None or not self.geometry.width*self.geometry.height:missing.append('RTAB-Map geometry cloud')
        if self.private_edges[self.c['frames']['map']+' -> '+self.c['frames']['odom']]<3:missing.append('RTAB-Map map -> odom')
        if self.tf_ok<3:missing.append('capture-time map <- camera TF')
        if self.grid.get('transformed',0)<1:missing.append('semantic map accumulation')
        if self.video_enabled and self.video_frames<1:missing.append('video recording')
        return missing
    def snapshot(self):
        state=dict(backend='rtabmap',namespace=self.c['namespace'],counts=dict(self.counts),
            sync_matches=self.sync_count,sync_max_spread_ms=max(self.sync_spreads,default=None),
            capture_tf_success=self.tf_ok,capture_tf_failed=self.tf_failed,last_tf_error=self.last_tf_error,
            private_tf_edges=dict(self.private_edges),graph_poses=self.graph_poses,loop_closures=self.loop_closures,
            geometry_points=self.geometry.width*self.geometry.height if self.geometry else 0,
            detector=self.detector,semantic_grid=self.grid,invalid_odom=self.invalid_odom,
            missing_for_readiness=self.readiness(initial=not self.initial_verified),fatal=self.fatal,video_frames=self.video_frames)
        self.csv.writerow([time.time(),self.get_clock().now().nanoseconds,*[self.counts[k] for k in ('rgb','depth')],
            self.sync_count,self.counts['rtabmap_info'],self.graph_poses,state['geometry_points'],self.tf_ok,self.tf_failed,
            self.counts['points_camera'],self.counts['points_map'],self.grid.get('voxels',0)])
        self.timeline.flush();atomic_json(self.output/'live_status.json',state);return state
    def close_artifacts(self):
        self.tf_executor.shutdown(timeout_sec=2);self.tf_thread.join(timeout=2)
        self.tf_listener.unregister();self.tf_node.destroy_node()
        self.video_stop.set()
        if self.video_thread:
            self.video_thread.join(timeout=20)
            if self.video_thread.is_alive():raise RuntimeError('Video encoder did not finish; output finalization is unconfirmed')
        if self.geometry:write_cloud(self.output/'rtabmap_geometry.ply',self.geometry)
        if self.occupancy:
            g=self.occupancy;a=np.array(g.data,np.int16).reshape(g.info.height,g.info.width)
            image=np.where(a<0,205,np.where(a>50,0,254)).astype('u1');cv2.imwrite(str(self.output/'occupancy.png'),np.flipud(image))
            atomic_json(self.output/'occupancy_origin.json',dict(frame=g.header.frame_id,resolution=g.info.resolution,
                x=g.info.origin.position.x,y=g.info.origin.position.y,orientation=[g.info.origin.orientation.x,
                g.info.origin.orientation.y,g.info.origin.orientation.z,g.info.origin.orientation.w]))
        for name,(stamp,image) in self.images.items():cv2.imwrite(str(self.output/('last_'+name+'.jpg')),image)
        atomic_json(self.output/'video_metadata.json',dict(path=str(self.video_path),fps=self.c['video']['fps'],
            frames=self.video_frames,skipped_schedule_ticks=self.video_skipped_ticks,
            encoded_duration_seconds=self.video_frames/self.c['video']['fps'],
            method='Hold latest actual RGB and YOLO output, no interpolation; encoder timestamps in video_frames.csv; missed schedule ticks are reported'))
        self.timeline.close()
