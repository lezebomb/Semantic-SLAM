"""Attach semantic nodes to registered raw RGB-D and an existing robot SLAM/TF stack."""
import datetime
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]


def launch_nodes(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    output = Path(get('output_dir')).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    model = Path(get('model_path')).expanduser().resolve()
    if not model.is_file():
        raise RuntimeError(f'Model checkpoint missing: {model}')
    python = os.environ.get('SEMANTIC_PYTHON', str(ROOT / '.venv/bin/python'))
    nodes = []
    for name in ('yolo_detector', 'semantic_projector', 'semantic_grid_node'):
        cmd = [python, str(ROOT / 'semantic' / f'{name}.py'), '--ros-args',
               '--params-file', get('params_file'), '-p', 'use_sim_time:=' + get('use_sim_time'),
               '-p', 'output_dir:=' + str(output)]
        if name == 'yolo_detector':
            cmd += ['-p', 'model_path:=' + str(model),
                    '-r', '/semantic/input/rgb:=' + get('rgb_topic'),
                    '-r', '/semantic/input/depth:=' + get('depth_topic'),
                    '-r', '/data_throttled_camera_info:=' + get('camera_info_topic')]
        nodes.append(ExecuteProcess(cmd=cmd, output='screen'))
    if get('rviz').lower() == 'true':
        nodes.append(ExecuteProcess(cmd=['rviz2', '-d', str(ROOT / 'config/semantic_indoor.rviz'),
            '--ros-args', '-p', 'use_sim_time:=' + get('use_sim_time')], output='screen'))
    return nodes


def generate_launch_description():
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    defaults = dict(rgb_topic='/camera/color/image_raw',
        depth_topic='/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/color/camera_info',
        model_path=str(ROOT / 'models/yolov8n-seg.pt'),
        params_file=str(ROOT / 'config/live.yaml'),
        output_dir=str(ROOT / 'runs' / ('live-' + stamp)), use_sim_time='false', rviz='true')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=launch_nodes)])
