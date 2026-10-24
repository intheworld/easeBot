"""Four Damiao VEL wheels on a dedicated USB-CAN bus; active by default."""
from pathlib import Path
import re

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
    profile = LaunchConfiguration('config_file').perform(context) or str(share / 'config/base.yaml')
    config = yaml.safe_load(Path(profile).read_text())
    expected = {f'{corner}_wheel_joint' for corner in ('front_left', 'front_right', 'rear_left', 'rear_right')}
    names = [m['name'] for m in config['motors']]
    if len(names) != 4 or set(names) != expected:
        raise ValueError('base profile must contain exactly the four wheel joints')
    readonly = LaunchConfiguration('read_only').perform(context)
    robot = xacro.process_file(str(Path(get_package_share_directory('hei_description')) /
        'urdf/hei_damiao_base.urdf.xacro'), mappings={
            'config_file': profile, 'read_only': readonly,
            'configure_control_mode': LaunchConfiguration('configure_control_mode').perform(context)}).toxml()
    controllers = str(Path(get_package_share_directory('hei_control')) / 'config/controllers.yaml')
    params = yaml.safe_load(Path(controllers).read_text())['/**/controller_manager']['ros__parameters']
    params['enforce_command_limits'] = False  # Hardware validates wheel velocity commands.
    params['base_joint_state_broadcaster'] = {'type': 'joint_state_broadcaster/JointStateBroadcaster'}
    remaps = [('robot_description', 'base_robot_description'), ('joint_states', 'base_joint_states'),
              ('/joint_states', 'base_joint_states'), ('dynamic_joint_states', 'base_dynamic_joint_states'),
              ('/dynamic_joint_states', 'base_dynamic_joint_states'), ('/tf', 'base_tf'), ('/tf_static', 'base_tf_static')]
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
        name='base_robot_state_publisher', parameters=[{'robot_description': robot}], remappings=remaps, output='screen')
    manager = Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
        name='base_controller_manager', parameters=[controllers, params], remappings=remaps, output='screen')
    manager_path = f'/{namespace}/base_controller_manager'
    states = Node(package='controller_manager', executable='spawner', namespace=namespace,
        arguments=['base_joint_state_broadcaster', '-c', manager_path], output='screen')
    args = ['base_controller', '-c', manager_path]
    if LaunchConfiguration('activate_base').perform(context) != 'true':
        args.append('--inactive')
    motion = Node(package='controller_manager', executable='spawner', namespace=namespace,
        arguments=args, output='screen')

    def after_states(event, _):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='base state broadcaster failed'))]
        return [motion] if readonly == 'false' else []

    actions = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='base service exited'))])) for node in (rsp, manager)]
    actions += [RegisterEventHandler(OnProcessExit(target_action=states, on_exit=after_states)),
        RegisterEventHandler(OnProcessExit(target_action=motion,
            on_exit=lambda event, _: [] if event.returncode == 0 else
                [EmitEvent(event=Shutdown(reason='base controller startup failed'))]))]
    return actions + [rsp, manager, states]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('read_only', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('activate_base', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('configure_control_mode', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
