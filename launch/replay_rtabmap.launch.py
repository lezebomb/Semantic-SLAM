"""Internal compatibility launch; playback/TF/recording are managed by run.sh --replay."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    arguments=dict(device='cpu',use_sim_time='true',rviz='false',
                   config=str(root/'config/robot_mapping.yaml'),output_dir=str(root/'runs/replay-manual'))
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in arguments.items()]+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/live_semantic.launch.py')),
            launch_arguments={k:LaunchConfiguration(k) for k in arguments}.items())])
