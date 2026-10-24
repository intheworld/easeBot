"""One real Damiao arm and gripper sharing one serial owner. Controllers are active by default."""
import math
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    arm = LaunchConfiguration('arm').perform(context)
    manager_name = LaunchConfiguration('controller_manager_name').perform(context)
    states_name = LaunchConfiguration('state_broadcaster_name').perform(context)
    hardware_name = LaunchConfiguration('hardware_name').perform(context)
    if any(not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', name)
           for name in (manager_name, states_name, hardware_name)):
        raise ValueError('manager, broadcaster and hardware names must be ROS identifiers')
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    hardware = Path(get_package_share_directory('hei_bringup'))
    config_file = LaunchConfiguration('config_file').perform(context) or str(hardware / 'config' / f'{arm}_arm.yaml')
    config = yaml.safe_load(Path(config_file).read_text())
    prefix = 'a_right' if arm == 'right' else 'b_left'
    expected = [f'{prefix}_joint{i}' for i in range(1, 7)]
    if [motor['name'] for motor in config['motors']] != expected:
        raise ValueError(f'arm={arm} requires these six joints in order: {expected}')
    include_gripper = LaunchConfiguration('include_gripper').perform(context) == 'true'
    if include_gripper and config.get('gripper', {}).get('name') != f'{arm}_gripper_motor':
        raise ValueError('gripper resource must match selected arm')
    arm_read_only = LaunchConfiguration('arm_read_only').perform(context) == 'true'
    robot = xacro.process_file(
        str(Path(get_package_share_directory('hei_description')) / 'urdf/hei_damiao_arm.urdf.xacro'),
        mappings={'config_file': config_file, 'read_only': LaunchConfiguration('read_only').perform(context),
                  'include_gripper': str(include_gripper).lower(),
                  'arm_read_only': str(arm_read_only).lower(), 'hardware_name': hardware_name,
                  'configure_control_mode': LaunchConfiguration('configure_control_mode').perform(context)}).toxml()
    controllers = str(Path(get_package_share_directory('hei_control')) / 'config/controllers.yaml')
    gripper_config = LaunchConfiguration('gripper_config_file').perform(context) or str(Path(get_package_share_directory('hei_control')) / 'config/damiao_gripper.yaml')
    manager_parameters = yaml.safe_load(Path(controllers).read_text())['/**/controller_manager']['ros__parameters']
    # The hardware adapter validates targets. The generic URDF limiter also
    # rejects measured startup positions outside the reference model range.
    manager_parameters['enforce_command_limits'] = False
    manager_parameters[states_name] = {'type': 'joint_state_broadcaster/JointStateBroadcaster'}
    parameters = [controllers, manager_parameters]
    writable = LaunchConfiguration('read_only').perform(context) == 'false'
    if include_gripper and writable:
        profile = yaml.safe_load(Path(gripper_config).read_text())
        node_name = f'{arm}_gripper_controller'
        selected = profile.get(f'/{namespace}/{node_name}', profile.get(f'/**/{node_name}', {}))
        settings = selected.get('ros__parameters', {})
        motor = config['gripper']
        if settings.get('motor_resource') != motor['name']:
            raise ValueError('gripper controller motor_resource must match hardware profile')
        for name, bound, upper in (
            ('max_velocity_rad_s', motor['velocity_limit'], True),
            ('max_current_ratio', motor['current_ratio_limit'], True),
            ('position_min_rad', motor['minimum'], False),
            ('position_max_rad', motor['maximum'], True)):
            value = settings.get(name)
            if value is None or not math.isfinite(float(value)) or (value > bound if upper else value < bound):
                raise ValueError(f'gripper controller {name} exceeds hardware limits')
    position_only = not writable or LaunchConfiguration('gripper_position_only').perform(context) == 'true'
    position_config = str(Path(get_package_share_directory('hei_control')) / 'config/gripper_position_only.yaml')
    if include_gripper:
        parameters += [gripper_config]
        if position_only:
            parameters += [position_config]
        parameters += [{f'{arm}_gripper_controller.type': 'hei_hardware/GripperController'}]
    manager_path = f'/{namespace}/{manager_name}'
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
               name=f'{arm}_robot_state_publisher',
               parameters=[{'robot_description': robot}], output='screen')
    control = Node(package='controller_manager', executable='ros2_control_node', namespace=namespace, name=manager_name,
                   parameters=parameters, output='screen')
    states = Node(package='controller_manager', executable='spawner', namespace=namespace,
                  arguments=[states_name, '-c', manager_path], output='screen')
    motion_args = [f'{arm}_arm_controller', '-c', manager_path]
    if LaunchConfiguration('activate_arm').perform(context) != 'true':
        motion_args.append('--inactive')
    motion = Node(package='controller_manager', executable='spawner', namespace=namespace,
                  arguments=motion_args, output='screen')
    gripper_args = [f'{arm}_gripper_controller', '-c', manager_path, '--param-file', gripper_config]
    if position_only:
        gripper_args += ['--param-file', position_config]
    if not writable or LaunchConfiguration('activate_gripper').perform(context) != 'true':
        gripper_args.append('--inactive')
    gripper = Node(package='controller_manager', executable='spawner', namespace=namespace,
                   arguments=gripper_args, output='screen')
    monitor = Node(package='hei_monitor', executable='monitor', namespace=namespace,
                   condition=IfCondition(LaunchConfiguration('monitor')), output='screen')
    actions = []
    for critical in (rsp, control):
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=critical, on_exit=[EmitEvent(event=Shutdown(reason='Damiao arm service exited'))])))

    def after_states(event, _):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='state broadcaster failed'))]
        if writable and not arm_read_only:
            return [motion]
        return [gripper] if include_gripper else []

    def after_motion(event, _):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='arm controller configuration failed'))]
        return [gripper] if include_gripper else []

    actions += [RegisterEventHandler(OnProcessExit(target_action=gripper, on_exit=lambda event, _: [] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='gripper controller configuration failed'))])),
                RegisterEventHandler(OnProcessExit(target_action=states, on_exit=after_states)),
                RegisterEventHandler(OnProcessExit(target_action=motion, on_exit=after_motion))]
    return actions + [rsp, control, states, monitor]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('arm', default_value='right', choices=['right', 'left']),
        DeclareLaunchArgument('controller_manager_name', default_value='controller_manager'),
        DeclareLaunchArgument('state_broadcaster_name', default_value='joint_state_broadcaster'),
        DeclareLaunchArgument('hardware_name', default_value='HeiDamiaoArm'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('arm_read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gripper_position_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('activate_arm', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('activate_gripper', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('include_gripper', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gripper_config_file', default_value=''),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('configure_control_mode', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('monitor', default_value='false'),
        OpaqueFunction(function=setup),
    ])
