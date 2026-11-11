#!/usr/bin/env python3
"""Passive control-chain and RTAB-Map monitoring; never publishes control."""
import argparse,csv,gzip,json,math,signal,time
from pathlib import Path
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rtabmap_msgs.msg import MapGraph
from mapping_config import ROOT,load_config
from mapping_runtime import atomic_json

class Monitor(Node):
 def __init__(self,out,c):
  super().__init__('gamepad_monitor',namespace=c['namespace'])
  self.out=out;self.c=c;self.counts=Counter();self.last={};self.nonzero=Counter();self.path=0.;self.previous=None;self.graph=0;self.subs=[]
  self.stream=gzip.open(out/'control_messages.jsonl.gz','wt',compresslevel=3)
  self.timeline=(out/'control_timeline.csv').open('w',newline='');self.csv=csv.writer(self.timeline)
  self.csv.writerow(['wall_time','A_enabled','raw_v','raw_w','final_v','final_w','wheel_v','wheel_w','wheel_path_estimate_m','rtabmap_graph_poses','drive_ready'])
  for typ,topic in [(Joy,'/joy'),(Twist,'/cmd_vel_raw'),(Twist,'/cmd_vel_smooth'),(Twist,'/cmd_vel_safe'),(Twist,'/cmd_vel'),(Odometry,c['topics']['odom']),(Bool,'/field/drive_ready')]:
   self.subs.append(self.create_subscription(typ,topic,self.callback(topic),qos_profile_sensor_data))
  self.subs.append(self.create_subscription(MapGraph,c['namespace']+'/mapGraph',self.callback(c['namespace']+'/mapGraph'),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
 def callback(self,topic):
  def receive(msg):
   self.counts[topic]+=1;data={}
   if isinstance(msg,Joy):data={'axes':list(msg.axes),'buttons':list(msg.buttons),'A_enabled':bool(msg.buttons and msg.buttons[0])}
   elif isinstance(msg,Twist):
    data={'v':msg.linear.x,'w':msg.angular.z};self.nonzero[topic]+=int(abs(msg.linear.x)>1e-5 or abs(msg.angular.z)>1e-5)
   elif isinstance(msg,Odometry):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation;xy=(p.x,p.y)
    if self.previous is not None:self.path+=math.hypot(xy[0]-self.previous[0],xy[1]-self.previous[1])
    self.previous=xy;data={'v':msg.twist.twist.linear.x,'w':msg.twist.twist.angular.z,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
    self.nonzero[topic]+=int(abs(data['v'])>1e-5 or abs(data['w'])>1e-5)
   elif isinstance(msg,Bool):data={'ready':bool(msg.data)}
   elif isinstance(msg,MapGraph):self.graph=len(msg.poses);data={'graph_poses':self.graph}
   if hasattr(msg,'header'):data['stamp_ns']=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
   data['received_wall_time']=time.time();self.last[topic]=data
   self.stream.write(json.dumps({'received_wall_time':time.time(),'topic':topic,**data})+'\n')
  return receive
 def save(self,running=True):
  joy=self.last.get('/joy',{});raw=self.last.get('/cmd_vel_raw',{});final=self.last.get('/cmd_vel',{});odom=self.last.get(self.c['topics']['odom'],{});ready=self.last.get('/field/drive_ready',{}).get('ready')
  mapping={}
  try:
   m=json.loads((self.out/'live_status.json').read_text());mapping={k:m.get(k) for k in ['counts','capture_tf_success','capture_tf_failed','geometry_points','graph_poses','missing_for_readiness']};mapping['semantic_voxels']=m.get('semantic_grid',{}).get('voxels')
  except (OSError,json.JSONDecodeError):pass
  state={'running':running,'updated_wall_time':time.time(),'control_publications':0,'counts':dict(self.counts),'nonzero_messages':dict(self.nonzero),'last':self.last,'wheel_path_estimate_m':self.path,'drive_ready':ready,'mapping':mapping}
  atomic_json(self.out/'gamepad_monitor.json',state)
  self.csv.writerow([time.time(),joy.get('A_enabled'),raw.get('v'),raw.get('w'),final.get('v'),final.get('w'),odom.get('v'),odom.get('w'),self.path,self.graph,ready]);self.timeline.flush();self.stream.flush()
 def close(self):self.save(False);self.timeline.close();self.stream.close()

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path);p.add_argument('--seconds',type=float,default=0);a=p.parse_args();out=a.output.resolve();assert (out/'configuration.json').is_file()
 c=load_config(a.config);rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Monitor(out,c);stop=[False]
 def halt(sig,frame):stop[0]=True
 for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,halt)
 start=time.monotonic();last=0
 try:
  while rclpy.ok() and not stop[0]:
   rclpy.spin_once(node,timeout_sec=.02)
   if time.monotonic()-last>=1:node.save();last=time.monotonic()
   if (out/'result.json').exists() or (a.seconds and time.monotonic()-start>=a.seconds):break
 finally:node.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
