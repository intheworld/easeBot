"""Three physical cameras, configured and activated without motor processes."""
from pathlib import Path
import os
import re
import sys

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def camera_environment():
    """Find workspace-local SDK even in terminals sourced before its installation."""
    environment = {'PYTHONNOUSERSITE': '1'}
    prefix = Path(get_package_prefix('hei_sensors')).resolve()
    for parent in (prefix, *prefix.parents):
        sdk = parent / '.deps/sensors-python'
        if sdk.is_dir():
            inherited = os.environ.get('PYTHONPATH', '')
            environment['PYTHONPATH'] = str(sdk) + (os.pathsep + inherited if inherited else '')
            break
    return environment


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    profile = LaunchConfiguration('config_file').perform(context) or str(
        Path(get_package_share_directory('hei_bringup')) / 'config/cameras.yaml')
    return [LifecycleNode(package='hei_sensors', executable='cameras', name='cameras',
        namespace=namespace, prefix=[sys.executable], output='screen',
        additional_env=camera_environment(),
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
