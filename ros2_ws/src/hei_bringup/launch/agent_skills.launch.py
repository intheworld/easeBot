"""Start only the skill bridge; default backend is explicitly simulated."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('hei_agent_bridge')) / 'config/skills.mock.json')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('hub_url', default_value='auto'),
        DeclareLaunchArgument('hub_id', default_value=''),
        DeclareLaunchArgument('discovery_interface', default_value=''),
        DeclareLaunchArgument('device_id', default_value='hei-01'),
        DeclareLaunchArgument('skill_config_file', default_value=config),
        Node(package='hei_agent_bridge', executable='agent_bridge', name='agent_bridge',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[{'hub_url': LaunchConfiguration('hub_url'),
                          'hub_id': LaunchConfiguration('hub_id'),
                          'discovery_interface': LaunchConfiguration('discovery_interface'),
                          'device_id': LaunchConfiguration('device_id'), 'skills_enabled': True,
                          'skill_config_file': LaunchConfiguration('skill_config_file')}]),
    ])
