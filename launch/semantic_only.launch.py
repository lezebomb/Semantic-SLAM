"""Standalone semantic nodes, standard image_transport decoding, no SLAM backend."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    defaults = dict(rgb_base='/data_throttled_image', depth_base='/data_throttled_image_depth',
                    camera_info='/data_throttled_camera_info', output_dir=str(root/'artifacts/manual'),
                    model_path=str(root/'models/yolov8n-seg.pt'), confidence='0.35',
                    process_every_n='3', image_size='640', cpu_threads='2',
                    reuse_rgbd='true')
    args = [DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()]
    config = {k:LaunchConfiguration(k) for k in defaults}
    nodes = [SetParameter(name='use_sim_time', value=True),
             Node(package='image_transport', executable='republish', name='semantic_rgb_decoder',
                  condition=UnlessCondition(config['reuse_rgbd']),
                  arguments=['compressed', 'raw'], output='screen',
                  remappings=[('in',config['rgb_base']),
                              ('in/compressed',[config['rgb_base'],'/compressed']),
                              ('out','/semantic/input/rgb')]),
             Node(package='image_transport', executable='republish', name='semantic_depth_decoder',
                  condition=UnlessCondition(config['reuse_rgbd']),
                  arguments=['compressedDepth', 'raw'], output='screen',
                  remappings=[('in',config['depth_base']),
                              ('in/compressedDepth',[config['depth_base'],'/compressedDepth']),
                              ('out','/semantic/input/depth')]),
             # Official rgbd_sync already decodes with the standard compressed plugins.
             # Reuse its RGBDImage instead of running duplicate decoder processes.
             Node(package='rtabmap_util', executable='rgbd_split', name='semantic_rgbd_split',
                  condition=IfCondition(config['reuse_rgbd']), output='screen',
                  parameters=[{'qos':1,'queue_sub':10,'queue_pub':10}],
                  remappings=[('/rgbd_image/rgb/image','/semantic/input/rgb'),
                              ('/rgbd_image/depth/image','/semantic/input/depth'),
                              ('/rgbd_image/rgb/camera_info','/semantic/input/camera_info'),
                              ('/rgbd_image/depth/camera_info','/semantic/input/depth_camera_info')])]
    for name in ('yolo_detector','semantic_projector','semantic_grid_node'):
        cmd = [str(root/'.venv/bin/python'),str(root/'semantic'/f'{name}.py'),
               '--ros-args','-p','use_sim_time:=true','-p',['output_dir:=',config['output_dir']]]
        if name == 'yolo_detector':
            for k in ('model_path','confidence','process_every_n','image_size','cpu_threads'):
                cmd += ['-p',[k+':=',config[k]]]
            cmd += ['-r',['/data_throttled_camera_info:=',config['camera_info']]]
        nodes.append(ExecuteProcess(cmd=cmd,output='screen'))
    return LaunchDescription(args+nodes)
