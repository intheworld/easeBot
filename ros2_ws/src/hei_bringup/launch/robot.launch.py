"""All real actuators: configure modes, home lift, and activate controllers."""
from pathlib import Path
import math
import re
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    share = Path(get_package_share_directory('hei_bringup'))
    namespace = LaunchConfiguration('namespace').perform(context)
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    readonly = LaunchConfiguration('read_only').perform(context)
    configure = LaunchConfiguration('configure_control_mode').perform(context)
    timeout = float(LaunchConfiguration('startup_timeout_sec').perform(context))
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('startup_timeout_sec must be positive and finite')
    profiles, ports = {}, {}
    for component, default in [('left', 'left_arm'), ('right', 'right_arm'), ('base', 'base'), ('lift', 'lift')]:
        path = LaunchConfiguration(f'{component}_config_file').perform(context) or str(share / f'config/{default}.yaml')
        profiles[component] = path
        config = yaml.safe_load(Path(path).read_text())
        for key in (('port', 'io_port') if component == 'lift' else ('port',)):
            device = str(Path(config[key]).resolve())
            if device in ports:
                raise ValueError(f'serial device conflict: {component}.{key} and {ports[device]} use {device}')
            ports[device] = f'{component}.{key}'

    common = {'namespace': namespace, 'read_only': readonly, 'configure_control_mode': configure}

    def include(filename, arguments):
        # Explicit scopes prevent config_file/activation arguments leaking between buses.
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / f'launch/{filename}')),
            launch_arguments={**common, **arguments}.items())], scoped=True)

    arms = include('damiao_arms.launch.py', {
        'left_config_file': profiles['left'], 'right_config_file': profiles['right'],
        'gripper_config_file': LaunchConfiguration('gripper_config_file').perform(context),
        'include_gripper': 'true', 'position_only': LaunchConfiguration('position_only').perform(context),
        'activate_arm': 'true', 'activate_gripper': 'true'})
    base = include('damiao_base.launch.py', {'config_file': profiles['base'], 'activate_base': 'true'})
    lift = include('damiao_lift.launch.py', {'config_file': profiles['lift'], 'home': 'true', 'activate_lift': 'true'})
    ready = Node(package='hei_bringup', executable='wait_robot_ready', namespace=namespace,
        prefix=[sys.executable], output='screen', parameters=[{
            'read_only': readonly == 'true',
            'timeout_sec': timeout}])
    gripper_profile = LaunchConfiguration('gripper_config_file').perform(context) or str(
        Path(get_package_share_directory('hei_control')) / 'config/damiao_gripper.yaml')
    grippers = yaml.safe_load(Path(gripper_profile).read_text())
    monitor_parameters = {'backend': 'damiao', 'read_only': readonly == 'true',
        'host': LaunchConfiguration('monitor_host'),
        'port': int(LaunchConfiguration('monitor_port').perform(context))}
    for side in ('left', 'right'):
        controller = f'{side}_gripper_controller'
        settings = grippers.get(f'/{namespace}/{controller}', grippers.get(f'/**/{controller}', {}))['ros__parameters']
        for key in ('gap_at_zero_m', 'gap_per_motor_rad_m'):
            monitor_parameters[f'{side}_{key}'] = float(settings[key])
    monitor = Node(package='hei_monitor', executable='monitor', namespace=namespace,
        output='screen', condition=IfCondition(LaunchConfiguration('monitor')),
        parameters=[monitor_parameters])
    cameras = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/cameras.launch.py')),
        launch_arguments={'namespace': namespace,
            'config_file': LaunchConfiguration('camera_config_file'),
            'backend': LaunchConfiguration('camera_backend')}.items())],
        scoped=True, condition=IfCondition(LaunchConfiguration('cameras')))
    on_ready = RegisterEventHandler(OnProcessExit(target_action=ready,
        on_exit=lambda event, _: [] if event.returncode == 0 else
            [EmitEvent(event=Shutdown(reason='robot actuator startup failed or timed out'))]))
    return [on_ready, arms, base, lift, ready, monitor, cameras]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('configure_control_mode', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('position_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('left_config_file', default_value=''),
        DeclareLaunchArgument('right_config_file', default_value=''),
        DeclareLaunchArgument('base_config_file', default_value=''),
        DeclareLaunchArgument('lift_config_file', default_value=''),
        DeclareLaunchArgument('gripper_config_file', default_value=''),
        DeclareLaunchArgument('startup_timeout_sec', default_value='90'),
        DeclareLaunchArgument('monitor', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('monitor_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('monitor_port', default_value='8081'),
        DeclareLaunchArgument('cameras', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera_config_file', default_value=''),
        DeclareLaunchArgument('camera_backend', default_value='opencv', choices=['opencv', 'mock']),
        OpaqueFunction(function=setup),
    ])
