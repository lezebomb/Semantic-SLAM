"""Include the unchanged official Indoor launch with an isolated database."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def prepare_output(context):
    output = Path(LaunchConfiguration('output_dir').perform(context)).expanduser().resolve()
    if (output/'rtabmap.db').exists():
        raise RuntimeError('Refusing to clear an existing database; choose a fresh output_dir: ' + str(output))
    output.mkdir(parents=True, exist_ok=True)
    return []


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    output = LaunchConfiguration('output_dir')
    official = Path(get_package_share_directory('rtabmap_demos'))/'launch/robot_mapping_demo.launch.py'
    names = ('rgb_base','depth_base','camera_info','model_path','confidence','process_every_n','image_size','cpu_threads')
    defaults = ('/data_throttled_image','/data_throttled_image_depth','/data_throttled_camera_info',
                str(root/'models/yolov8n-seg.pt'),'0.35','3','640','2')
    args = [DeclareLaunchArgument('output_dir',default_value=str(root/'artifacts/manual')),
            DeclareLaunchArgument('rviz',default_value='true')]
    args += [DeclareLaunchArgument(k,default_value=v) for k,v in zip(names,defaults)]
    semantic_args = {k:LaunchConfiguration(k) for k in names}
    semantic_args['output_dir'] = output
    return LaunchDescription(args+[
        OpaqueFunction(function=prepare_output),
        GroupAction([
            SetParameter(name='database_path',value=[output,'/rtabmap.db']),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(official)),
                launch_arguments={'rviz':'false','rtabmap_viz':'false'}.items())]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/semantic_only.launch.py')),
                                 launch_arguments=semantic_args.items()),
        Node(package='rviz2', executable='rviz2', name='semantic_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time':True}],
             arguments=['-d',str(root/'config/semantic_indoor.rviz')])])
