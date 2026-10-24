"""Lift-only bus and passive limit IO. Homes and activates by default."""
from pathlib import Path
import re
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    share = Path(get_package_share_directory('hei_bringup'))
    profile = LaunchConfiguration('config_file').perform(context) or str(share / 'config/lift.yaml')
    settings = yaml.safe_load(Path(profile).read_text())
    readonly = LaunchConfiguration('read_only').perform(context) == 'true'
    home = LaunchConfiguration('home').perform(context) == 'true' and not readonly
    activate = LaunchConfiguration('activate_lift').perform(context) == 'true'
    robot = xacro.process_file(str(Path(get_package_share_directory('hei_description')) /
        'urdf/hei_damiao_lift.urdf.xacro'), mappings={
            'config_file': profile, 'read_only': str(readonly).lower(), 'home': str(home).lower(),
            'configure_control_mode': LaunchConfiguration('configure_control_mode').perform(context)}).toxml()
    controllers = str(Path(get_package_share_directory('hei_control')) / 'config/controllers.yaml')
    params = yaml.safe_load(Path(controllers).read_text())['/**/controller_manager']['ros__parameters']
    lift_parameters = str(Path(get_package_share_directory('hei_control')) / 'config/damiao_lift.yaml')
    params['enforce_command_limits'] = False
    params['lift_controller'] = {'type': 'hei_hardware/LiftTrajectoryController'}
    params['lift_joint_state_broadcaster'] = {'type': 'joint_state_broadcaster/JointStateBroadcaster'}
    remaps = [('robot_description', 'lift_robot_description'), ('joint_states', 'lift_joint_states'),
              ('/joint_states', 'lift_joint_states'), ('dynamic_joint_states', 'lift_dynamic_joint_states'),
              ('/dynamic_joint_states', 'lift_dynamic_joint_states'), ('/tf', 'lift_tf'), ('/tf_static', 'lift_tf_static')]
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
        name='lift_robot_state_publisher', parameters=[{'robot_description': robot}], remappings=remaps, output='screen')
    manager = Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
        name='lift_controller_manager', parameters=[controllers, lift_parameters, params], remappings=remaps, output='screen')
    manager_path = f'/{namespace}/lift_controller_manager'
    manager_timeout = str(max(10.0, float(settings['io_wait_timeout_sec']) + float(settings['configure_timeout_ms'])/1000 + 5.0))
    states = Node(package='controller_manager', executable='spawner', namespace=namespace,
        arguments=['lift_joint_state_broadcaster', '-c', manager_path, '--controller-manager-timeout', manager_timeout], output='screen')
    args = ['lift_controller', '-c', manager_path, '--controller-manager-timeout', manager_timeout]
    if not home or not activate:
        args.append('--inactive')
    motion = Node(package='controller_manager', executable='spawner', namespace=namespace,
        arguments=args, output='screen')
    waiter = Node(package='hei_bringup', executable='wait_lift_ready', namespace=namespace,
        # ROS CLI uses its matching Python ABI; env python3 may select Conda.
        prefix=[sys.executable],
        parameters=[{'timeout_sec': float(settings['home_timeout_sec']) + float(settings['io_wait_timeout_sec']) + 5.0}], output='screen')

    def after_states(event, _):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='lift state broadcaster failed'))]
        if readonly:
            return []
        return [waiter] if home and activate else [motion]

    def after_ready(event, _):
        return [motion] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='lift homing failed'))]

    actions = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='lift service exited'))])) for node in (rsp, manager)]
    actions += [RegisterEventHandler(OnProcessExit(target_action=states, on_exit=after_states)),
        RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=after_ready)),
        RegisterEventHandler(OnProcessExit(target_action=motion,
            on_exit=lambda event, _: [] if event.returncode == 0 else
                [EmitEvent(event=Shutdown(reason='lift controller startup failed'))]))]
    return actions + [rsp, manager, states]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('home', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('activate_lift', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('configure_control_mode', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
