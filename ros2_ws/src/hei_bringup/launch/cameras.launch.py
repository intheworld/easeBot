"""Three physical cameras, configured and activated without motor processes."""
from pathlib import Path
import re
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    profile = LaunchConfiguration('config_file').perform(context) or str(
        Path(get_package_share_directory('hei_bringup')) / 'config/cameras.yaml')
    return [LifecycleNode(package='hei_sensors', executable='cameras', name='cameras',
        namespace=namespace, prefix=[sys.executable], output='screen',
        additional_env={'PYTHONNOUSERSITE': '1'},
        parameters=[profile, {'backend': LaunchConfiguration('backend')}]),
        Node(package='hei_bringup', executable='activate_cameras', namespace=namespace,
            prefix=[sys.executable], output='screen')]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('backend', default_value='opencv', choices=['opencv', 'mock']),
        OpaqueFunction(function=setup),
    ])
