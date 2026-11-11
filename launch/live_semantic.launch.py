"""RTAB-Map + YOLO full pipeline; run.sh manages inputs, TF and recording."""
import datetime
import json
import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from mapping_config import load_config, tf_topics, rtabmap_parameters, visual_odometry_parameters

def launch_nodes(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    c = load_config(get('config'))
    out = Path(get('output_dir')).expanduser().resolve()
    if (out/'rtabmap.db').exists():
        raise RuntimeError('Existing rtabmap.db is protected; choose a fresh output_dir')
    out.mkdir(parents=True, exist_ok=True)
    semantic = out/'semantic'; semantic.mkdir(exist_ok=True)
    model = Path(get('model_path')).expanduser().resolve()
    if not model.is_file(): raise RuntimeError('Missing YOLO checkpoint: '+str(model))
    sim = get('use_sim_time').lower() == 'true'
    tf, static = tf_topics(c); t = c['topics']
    geometry = Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        namespace=c['namespace'], output='screen', sigterm_timeout='20', sigkill_timeout='5',
        parameters=[rtabmap_parameters(c,out,sim)],
        remappings=[('/tf',tf),('/tf_static',static),('rgb/image',t['rgb']),
                    ('depth/image',t['depth']),('rgb/camera_info',t['camera_info']),
                    ('odom',t['odom']),('scan',t['scan'])])
    odometry = Node(package='rtabmap_odom',executable='rgbd_odometry',name='rgbd_odometry',
        namespace=c['namespace'],output='screen',sigterm_timeout='20',sigkill_timeout='5',
        parameters=[visual_odometry_parameters(c,sim)],
        remappings=[('/tf',tf),('/tf_static',static),('rgb/image',t['rgb']),
                    ('depth/image',t['depth']),('rgb/camera_info',t['camera_info']),
                    ('odom',t['odom'])])
    nodes = [odometry,geometry]
    laser_tf = Node(package='tf2_ros', executable='static_transform_publisher',
        name='camera_to_laser_tf', namespace=c['namespace'], output='screen',
        arguments=['--x', '0.211', '--y', '0.1066', '--z', '-0.6',
                   '--qx', '0', '--qy', '0', '--qz', '-0.082905', '--qw', '0.996557',
                   '--frame-id', 'camera_link', '--child-frame-id', 'laser'])
    nodes.append(laser_tf)
    python = os.environ.get('SEMANTIC_PYTHON', str(ROOT/'.venv/bin/python'))
    for name in ('yolo_detector','semantic_projector','semantic_grid_node'):
        cmd = [python,str(ROOT/'semantic'/f'{name}.py'),'--ros-args',
               '--params-file',get('params_file'),'-p','use_sim_time:='+str(sim).lower(),
               '-p','output_dir:='+str(semantic),'-r','/tf:='+tf,'-r','/tf_static:='+static]
        if name == 'yolo_detector':
            cmd += ['-p','model_path:='+str(model),'-p','device:='+json.dumps(get('device')),
                    '-r','/semantic/input/rgb:='+t['rgb'],'-r','/semantic/input/depth:='+t['depth'],
                    '-r','/data_throttled_camera_info:='+t['camera_info']]
        if name == 'semantic_grid_node': cmd += ['-p','map_frame:='+c['frames']['map']]
        nodes.append(ExecuteProcess(cmd=cmd, output='screen', sigterm_timeout='20', sigkill_timeout='5'))
    result = list(nodes)
    for action in nodes:
        result.append(RegisterEventHandler(OnProcessExit(target_action=action,
            on_exit=[EmitEvent(event=Shutdown(reason='Required RTAB-Map/YOLO process exited'))])))
    if get('rviz').lower() == 'true':
        result.append(Node(package='rviz2',executable='rviz2',output='screen',
            arguments=['-d',str(ROOT/'config/robot_mapping.rviz')],
            parameters=[{'use_sim_time':sim}], remappings=[('/tf',tf),('/tf_static',static)]))
    return result

def generate_launch_description():
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    defaults = dict(config=str(ROOT/'config/robot_mapping.yaml'),
        model_path=str(ROOT/'models/yolov8n-seg.pt'),params_file=str(ROOT/'config/live.yaml'),
        output_dir=str(ROOT/'runs'/('live-'+stamp)),use_sim_time='false',device='0',rviz='false')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
                             + [OpaqueFunction(function=launch_nodes)])
