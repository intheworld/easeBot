"""Both grippers, one ROS namespace with distinct managers and serial owners."""
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
import yaml


def setup(context):
    share = Path(get_package_share_directory('hei_bringup'))
    namespace = LaunchConfiguration('namespace').perform(context)
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    sides = ('left', 'right')
    profiles = {side: LaunchConfiguration(f'{side}_config_file').perform(context) or
                str(share / 'config' / f'{side}_arm.yaml') for side in sides}
    ports = [Path(yaml.safe_load(Path(profiles[side]).read_text())['port']).resolve()
             for side in sides]
    if ports[0] == ports[1]:
        raise ValueError('left and right profiles must use different serial devices')
    shared = {name: LaunchConfiguration(name).perform(context)
              for name in ('read_only', 'activate_gripper', 'gripper_config_file')}
    shared['gripper_position_only'] = LaunchConfiguration('position_only').perform(context)
    return [GroupAction([
        # Share /hei for public APIs; give per-bus descriptions and state trees unique topics.
        SetRemap(src='robot_description', dst=f'{side}_robot_description'),
        SetRemap(src='joint_states', dst=f'{side}_joint_states'),
        SetRemap(src='/joint_states', dst=f'{side}_joint_states'),
        SetRemap(src='dynamic_joint_states', dst=f'{side}_dynamic_joint_states'),
        SetRemap(src='/dynamic_joint_states', dst=f'{side}_dynamic_joint_states'),
        SetRemap(src='/tf', dst=f'{side}_tf'),
        SetRemap(src='/tf_static', dst=f'{side}_tf_static'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/damiao_arm.launch.py')),
            launch_arguments={**shared, 'arm': side, 'namespace': namespace,
                              'controller_manager_name': f'{side}_controller_manager',
                              'state_broadcaster_name': f'{side}_joint_state_broadcaster',
                              'hardware_name': f'HeiDamiao{side.title()}',
                              'arm_read_only': 'true', 'include_gripper': 'true',
                              'monitor': 'false', 'config_file': profiles[side]}.items()),
    ], scoped=True) for side in sides]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('left_config_file', default_value=''),
        DeclareLaunchArgument('right_config_file', default_value=''),
        DeclareLaunchArgument('gripper_config_file', default_value=''),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('position_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('activate_gripper', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
