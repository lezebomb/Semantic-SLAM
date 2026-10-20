#!/usr/bin/env python3
"""Independent live evidence: image/mask/cloud/map counts and numeric TF check."""
import json
import signal
import sys
from pathlib import Path
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from rtabmap_msgs.msg import MapData, MapGraph
from std_msgs.msg import String
import tf2_ros
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'semantic'))
from common import POINT_DTYPE, rotation_matrix


class Observer(Node):
    def __init__(self, output, allow_empty=False):
        super().__init__('semantic_verifier', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = output
        self.allow_empty = allow_empty
        self.latest_header = None
        self.last_tf_stamp = None
        self.pipeline_tf_checks = 0
        self.bridge = CvBridge()
        self.stats = dict(topics={}, mask_nonzero_messages=0, instance_nonzero_messages=0,
                          semantic_cloud_nonempty=0, semantic_map_nonempty=0,
                          finite_clouds=0, tf_checks=0, max_tf_point_error_m=0.0,
                          detector_frames_with_instances=0, camera_frames={}, depth_encodings={})
        self.camera_clouds = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer,self)
        self.subs = []
        for topic, typ in [('/semantic/input/rgb',Image),('/semantic/input/depth',Image),
                           ('/data_throttled_camera_info',CameraInfo),
                           ('/semantic/image_annotated',Image),('/semantic/label_image',Image),
                           ('/semantic/instance_image',Image),('/semantic/points_camera',PointCloud2),
                           ('/semantic/pointcloud',PointCloud2),('/semantic/detector_status',String),
                           ('/semantic/map_status',String),('/map',OccupancyGrid),
                           ('/mapData',MapData),('/mapGraph',MapGraph),('/cloud_map',PointCloud2),
                           ('/semantic/map',PointCloud2)]:
            qos = (QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == '/semantic/map' else
                   QoSProfile(depth=10) if typ in (Image,CameraInfo,String)
                   else qos_profile_sensor_data)
            self.subs.append(self.create_subscription(typ,topic,lambda m,t=topic:self.receive(t,m),qos))
        self.create_timer(3.0,self.save)

    def receive(self, topic, msg):
        item = self.stats['topics'].setdefault(topic,dict(count=0))
        item['count'] += 1
        if hasattr(msg,'header'):
            item['frame_id'] = msg.header.frame_id
        if isinstance(msg,Image):
            if topic == '/semantic/image_annotated':
                self.latest_header = msg.header
            item.update(width=msg.width,height=msg.height,encoding=msg.encoding)
            if topic == '/semantic/input/depth':
                self.stats['depth_encodings'][msg.encoding] = item['count']
            if topic in ('/semantic/label_image','/semantic/instance_image'):
                a=self.bridge.imgmsg_to_cv2(msg,'passthrough')
                item['last_nonzero_pixels']=int(np.count_nonzero(a))
                key='mask_nonzero_messages' if topic.endswith('label_image') else 'instance_nonzero_messages'
                self.stats[key]+=int(np.any(a))
            if topic == '/semantic/image_annotated' and item['count']%25==1:
                cv2.imwrite(str(self.output/'observed_annotated.jpg'),self.bridge.imgmsg_to_cv2(msg,'bgr8'))
        if isinstance(msg,PointCloud2):
            item['points']=msg.width*msg.height
            item['fields']=[f.name for f in msg.fields]
            if topic.startswith('/semantic/'):
                p=np.frombuffer(msg.data,dtype=POINT_DTYPE)
                item['class_ids']=[int(x) for x in np.unique(p['class_id'])]
                self.stats['finite_clouds']+=int(all(np.isfinite(p[n]).all() for n in ('x','y','z')))
                stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                if topic.endswith('points_camera'):
                    self.camera_clouds[stamp]=msg
                    while len(self.camera_clouds)>40:
                        self.camera_clouds.pop(next(iter(self.camera_clouds)))
                if topic == '/semantic/pointcloud':
                    self.stats['semantic_cloud_nonempty']+=int(len(p)>0)
                    camera=self.camera_clouds.get(stamp)
                    if camera is not None:
                        try:
                            tf=self.tf_buffer.lookup_transform('map',camera.header.frame_id,Time.from_msg(camera.header.stamp))
                            raw=np.frombuffer(camera.data,dtype=POINT_DTYPE)
                            xyz=np.column_stack([raw[n] for n in ('x','y','z')])
                            t=tf.transform.translation
                            expected=xyz@rotation_matrix(tf.transform.rotation).T+np.array([t.x,t.y,t.z])
                            actual=np.column_stack([p[n] for n in ('x','y','z')])
                            error=float(np.max(np.abs(expected-actual)))
                            self.stats['tf_checks']+=1
                            self.stats['max_tf_point_error_m']=max(error,self.stats['max_tf_point_error_m'])
                        except tf2_ros.TransformException:
                            pass
                if topic == '/semantic/map':
                    self.stats['semantic_map_nonempty']+=int(len(p)>0)
        if isinstance(msg,OccupancyGrid):
            item.update(width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,
                        known_cells=int(np.count_nonzero(np.asarray(msg.data)>=0)))
        if isinstance(msg,MapData):
            item['nodes_in_message']=len(msg.nodes)
        if isinstance(msg,MapGraph):
            item['poses']=len(msg.poses)
        if isinstance(msg,String):
            item['last']=json.loads(msg.data)
            if topic == '/semantic/detector_status':
                self.stats['detector_frames_with_instances']+=int(bool(item['last']['detections']))

    def save(self):
        if self.latest_header is not None:
            stamp = (self.latest_header.stamp.sec, self.latest_header.stamp.nanosec)
            if stamp != self.last_tf_stamp:
                try:
                    tf = self.tf_buffer.lookup_transform('map', self.latest_header.frame_id,
                                                        Time.from_msg(self.latest_header.stamp))
                    t, q = tf.transform.translation, tf.transform.rotation
                    if all(np.isfinite(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)):
                        self.pipeline_tf_checks += 1
                        self.last_tf_stamp = stamp
                except tf2_ros.TransformException:
                    pass
        if rclpy.ok():
            self.stats['nodes']=self.get_node_names()
            self.stats['ros_topics']=[dict(name=n,types=t) for n,t in self.get_topic_names_and_types()]
        self.stats['success']=all((self.stats['mask_nonzero_messages']>0,
            self.stats['instance_nonzero_messages']>0,self.stats['semantic_cloud_nonempty']>0,
            self.stats['semantic_map_nonempty']>0,self.stats['tf_checks']>0,
            self.stats['topics'].get('/semantic/image_annotated',{}).get('count',0)>0,
            self.stats['detector_frames_with_instances']>0,
            self.stats['max_tf_point_error_m']<1e-4,
            self.stats['topics'].get('/map',{}).get('known_cells',0)>0,
            self.stats['topics'].get('/mapData',{}).get('count',0)>0))
        if self.allow_empty:
            topics = self.stats['topics']
            required = ('/semantic/input/rgb', '/semantic/input/depth', '/semantic/image_annotated',
                        '/semantic/label_image', '/semantic/instance_image', '/semantic/detector_status',
                        '/mapData')
            self.stats['success'] = all(topics.get(t, {}).get('count', 0) > 0 for t in required)
            self.stats['success'] &= (self.pipeline_tf_checks > 0
                and topics.get('/map', {}).get('known_cells', 0) > 0
                and topics.get('/mapGraph', {}).get('poses', 0) > 0
                and self.stats['max_tf_point_error_m'] < 1e-4)
        self.stats['verification_policy'] = 'interface_allow_empty' if self.allow_empty else 'nonempty_semantics'
        self.stats['pipeline_tf_checks'] = self.pipeline_tf_checks
        self.stats['segmentation_state'] = ('predictions_present' if self.stats['detector_frames_with_instances']
            else 'normal_empty_predictions' if self.stats['topics'].get('/semantic/detector_status', {}).get('count', 0)
            else 'waiting')
        self.stats['semantic_transform_validation'] = ('exercised_on_real_model_outputs'
            if self.stats['tf_checks'] else 'not_exercised')
        (self.output/'verification.json').write_text(json.dumps(self.stats,indent=2))


if __name__ == '__main__':
    rclpy.init()
    node=Observer(Path(sys.argv[1]), allow_empty='--allow-empty' in sys.argv[2:])
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
