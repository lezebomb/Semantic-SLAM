#!/usr/bin/env python3
"""Discover and subscribe only: no drivers, control publications or parameter changes."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

parser=argparse.ArgumentParser()
parser.add_argument('--seconds',type=float,default=10)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args()
assert 0<a.seconds<=30
rclpy.init(args=['--ros-args','--disable-rosout-logs','--disable-external-lib-logs'])
n=Node('deployment_readonly_interfaces',enable_rosout=False)
report=dict(domain_id=rclpy.get_default_context().get_domain_id(),
    ros_localhost_only=os.environ.get('ROS_LOCALHOST_ONLY','unset'),topics={},tf_edges={})
subscriptions={}
types={'sensor_msgs/msg/Image','sensor_msgs/msg/CompressedImage','sensor_msgs/msg/CameraInfo',
       'sensor_msgs/msg/LaserScan','sensor_msgs/msg/PointCloud2','nav_msgs/msg/Odometry','tf2_msgs/msg/TFMessage'}


def receive(topic,msg):
    item=report['topics'][topic]
    item['messages']=item.get('messages',0)+1
    if hasattr(msg,'header'):
        h=msg.header
        item.update(frame_id=h.frame_id,last_stamp=dict(sec=h.stamp.sec,nanosec=h.stamp.nanosec))
    for key in ('width','height','encoding','format'):
        if hasattr(msg,key):item[key]=getattr(msg,key)
    if hasattr(msg,'k'):
        item.update(k=list(msg.k),d=list(msg.d),distortion_model=msg.distortion_model)
    if hasattr(msg,'child_frame_id'):item['child_frame_id']=msg.child_frame_id
    if hasattr(msg,'transforms'):
        for t in msg.transforms:
            q=t.transform.rotation;p=t.transform.translation
            report['tf_edges'][t.header.frame_id+' -> '+t.child_frame_id]=dict(
                topic=topic,stamp=dict(sec=t.header.stamp.sec,nanosec=t.header.stamp.nanosec),
                translation=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w])


try:
    until=time.monotonic()+a.seconds
    while time.monotonic()<until:
        discovered = dict(n.get_topic_names_and_types())
        for topic,typs in discovered.items():
            if topic in subscriptions or not any(t in types for t in typs):continue
            typ=next(t for t in typs if t in types)
            # Do not request JPEG of 16UC1 depth or compressedDepth of RGB.
            # image_transport advertises these plugin endpoints even when unsuitable.
            if typ == 'sensor_msgs/msg/CompressedImage' and topic.rsplit('/', 1)[0] in discovered:
                continue
            info=[]
            for ep in n.get_publishers_info_by_topic(topic):
                info.append(dict(node=ep.node_namespace.rstrip('/')+'/'+ep.node_name,
                    reliability=str(ep.qos_profile.reliability),durability=str(ep.qos_profile.durability)))
            report['topics'][topic]=dict(type=typ,publishers=info,messages=0)
            qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
            if topic.endswith('tf_static'):
                qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            try:subscriptions[topic]=n.create_subscription(get_message(typ),topic,lambda m,t=topic:receive(t,m),qos)
            except Exception as e:report['topics'][topic]['error']=str(e)
        rclpy.spin_once(n,timeout_sec=.2)
    report['nodes']=[dict(name=x,namespace=y) for x,y in n.get_node_names_and_namespaces() if x!=n.get_name()]
    report['all_topics']=[dict(name=x,types=y) for x,y in n.get_topic_names_and_types()]
finally:
    n.destroy_node();rclpy.shutdown()
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
