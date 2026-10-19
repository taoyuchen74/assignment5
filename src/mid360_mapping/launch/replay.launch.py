"""Replay the recorded MID-360 bag through FAST-LIO and map processing."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    bag = Path(LaunchConfiguration('bag_path').perform(context)).expanduser().resolve()
    rate = float(LaunchConfiguration('rate').perform(context))
    if not bag.is_dir() or not (bag / 'metadata.yaml').is_file():
        raise RuntimeError(
            'bag_path must be a ROS 2 bag directory containing metadata.yaml')
    if not math.isfinite(rate) or rate <= 0:
        raise RuntimeError('rate must be finite and positive')

    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    imu_topic = LaunchConfiguration('imu_topic').perform(context)
    topics = [lidar_topic, imu_topic]
    if not all(
        isinstance(topic, str) and topic.startswith('/') and topic != '/clock'
        for topic in topics
    ):
        raise RuntimeError('lidar_topic and imu_topic must be absolute topic names')

    share = Path(get_package_share_directory('mid360_mapping'))
    command = ['ros2', 'bag', 'play', str(bag), '--clock', '60', '--rate', str(rate)]
    paused = LaunchConfiguration('start_paused').perform(context) == 'true'
    if paused:
        command.append('--start-paused')
    else:
        command += ['--delay', '3']
    command += ['--topics', *topics]

    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/mapping.launch.py')),
        launch_arguments={
            'fastlio_params_file':
                LaunchConfiguration('fastlio_params_file').perform(context),
            'params_file': LaunchConfiguration('params_file').perform(context),
            'map_params_file': LaunchConfiguration('map_params_file').perform(context),
            'map_output_path': LaunchConfiguration('map_output_path').perform(context),
            'rviz': LaunchConfiguration('rviz').perform(context),
        }.items())
    return [
        include,
        ExecuteProcess(cmd=command, output='screen', emulate_tty=True),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('mid360_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value=str(
                Path.home() / 'bags/assignment5/rosbag/'
                'rosbag2_2026_05_28-17_11_13')),
        DeclareLaunchArgument(
            'fastlio_params_file',
            default_value=str(share / 'config/fastlio_mid360.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(share / 'config/mid360_recorded.yaml')),
        DeclareLaunchArgument(
            'map_params_file',
            default_value=str(share / 'config/map_processing.yaml')),
        DeclareLaunchArgument(
            'map_output_path', default_value='maps/fastlio_map_from_bag.pcd'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar_192_168_1_140'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu_192_168_1_140'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('start_paused', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
