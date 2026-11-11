"""Regression checks for backend ownership, TF isolation and empty-scene health."""
import copy
from collections import Counter
from pathlib import Path
import sys,time,unittest
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from mapping_config import load_config,input_topics,record_topics,rtabmap_parameters

class ConfigurationTests(unittest.TestCase):
 def setUp(self):self.config=load_config()
 def test_replay_never_reuses_old_map_or_controls(self):
  inputs=input_topics(self.config)
  for forbidden in ['/tf','/fusion/odometry','/cmd_vel','/semantic/map','/semantic/pointcloud']:
   self.assertNotIn(forbidden,inputs)
  self.assertNotIn(self.config['topics']['odom'],inputs);self.assertIn('/tf_static',inputs)
 def test_geometry_ownership_cannot_be_overridden_by_config(self):
  c=copy.deepcopy(self.config);c['rtabmap'].update(publish_tf=False,odom_frame_id='fusion_odom',subscribe_depth=False)
  p=rtabmap_parameters(c,Path('/tmp/test-output'),True)
  self.assertTrue(p['publish_tf']);self.assertTrue(p['subscribe_depth']);self.assertEqual(p['odom_frame_id'],'')
  self.assertEqual(p['database_path'],'/tmp/test-output/rtabmap.db')
 def test_recording_does_not_mix_global_dynamic_tf(self):
  topics=record_topics(self.config)
  self.assertNotIn('/tf',topics)
  self.assertIn('/rtabmap_yolo/tf',topics)
  self.assertNotIn('/fusion/odometry',topics)
  self.assertNotIn('/cmd_vel',topics)

class RuntimePolicyTests(unittest.TestCase):
 def test_numeric_gpu_id_remains_a_ros_string(self):
  # ROS parses bare device:=0 as INTEGER; the detector requires STRING.
  import os,rclpy,importlib.util,tempfile
  from launch import LaunchContext
  from launch.actions import ExecuteProcess
  from launch.utilities import perform_substitutions
  from rclpy.context import Context
  from rclpy.node import Node
  root=Path(__file__).resolve().parents[1]
  spec=importlib.util.spec_from_file_location('live_launch',root/'launch/live_semantic.launch.py')
  module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
  os.environ.setdefault('ROS_LOG_DIR','/tmp/rtabmap-unit-logs')
  with tempfile.TemporaryDirectory() as output:
   launch_context=LaunchContext()
   launch_context.launch_configurations.update(config=str(root/'config/robot_mapping.yaml'),output_dir=output,
       model_path=str(root/'models/yolov8n-seg.pt'),params_file=str(root/'config/live.yaml'),
       use_sim_time='false',device='0',rviz='false')
   commands=[[perform_substitutions(launch_context,part) for part in action.cmd]
             for action in module.launch_nodes(launch_context) if type(action) is ExecuteProcess]
   command=next(cmd for cmd in commands if any('yolo_detector.py' in arg for arg in cmd))
   device_arg=next(arg for arg in command if arg.startswith('device:='))
  self.assertEqual(device_arg,'device:="0"')
  self.assertFalse(any('chassis_odom' in ' '.join(cmd) or 'icp_odometry' in ' '.join(cmd) for cmd in commands))
 def test_external_map_tf_is_never_imported(self):
  from mapping_runtime import Runtime
  from geometry_msgs.msg import TransformStamped
  from tf2_msgs.msg import TFMessage
  class Bridge:
   static={};published=False
   c=load_config()
   def publish_static(self):self.published=True
  bridge=Bridge();bad=TransformStamped();bad.header.frame_id='map';bad.child_frame_id='odom'
  good=TransformStamped();good.header.frame_id='camera_link';good.child_frame_id='camera_color_frame'
  Runtime.camera_static(bridge,TFMessage(transforms=[bad,good]))
  self.assertNotIn('odom',bridge.static);self.assertIn('camera_color_frame',bridge.static)
 def test_empty_scene_after_initial_check_does_not_stop_geometry(self):
  from mapping_runtime import Runtime
  class Health:pass
  state=Health();state.c=load_config();state.counts=Counter();state.arrivals={}
  for k in ['rgb','depth','camera_info','scan','encoder','valid_odom','rtabmap_info','detector','map_tf','capture_tf']:
   state.counts[k]=1;state.arrivals[k]=time.monotonic()
  state.sync_count=10;state.graph_poses=1
  class Cloud:width=20;height=1
  state.geometry=Cloud();state.private_edges=Counter({state.c['frames']['map']+' -> '+state.c['frames']['odom']:20});state.tf_ok=10;state.grid={'transformed':5}
  state.video_enabled=False;state.video_frames=0
  self.assertEqual(Runtime.readiness(state,initial=False),[])
  self.assertIn('points_camera',Runtime.readiness(state,initial=True))
  state.arrivals['rtabmap_info']=time.monotonic()-20
  self.assertIn('rtabmap_info',Runtime.readiness(state,initial=False))

if __name__=='__main__':unittest.main()
