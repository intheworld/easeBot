"""Native ROS 2 mock bringup. There is deliberately no real-hardware launch switch."""
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

MOTION = ['right_arm_controller', 'left_arm_controller', 'right_gripper_controller',
          'left_gripper_controller', 'lift_controller', 'base_controller']


def setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    if not re.fullmatch(r'[a-zA-Z][a-zA-Z0-9_]*', namespace):
        raise ValueError('namespace must be a single nonempty ROS identifier')
    description = Path(get_package_share_directory('hei_description'))
    robot = xacro.process_file(str(description / 'urdf/hei.urdf.xacro')).toxml()
    config = str(Path(get_package_share_directory('hei_control')) / 'config/controllers.yaml')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
               parameters=[{'robot_description': robot}], output='screen')
    control = Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
                   parameters=[config], output='screen')
    manager_path = '/' + namespace + '/controller_manager'
    states = Node(package='controller_manager', executable='spawner', namespace=namespace,
                  arguments=['joint_state_broadcaster', '--controller-manager', manager_path,
                             '--controller-manager-timeout', '30'], output='screen')
    motion = Node(package='controller_manager', executable='spawner', namespace=namespace,
                  arguments=MOTION + ['--inactive', '--controller-manager', manager_path,
                                      '--controller-manager-timeout', '30'], output='screen')
    manager = Node(package='hei_robot_manager', executable='robot_manager', namespace=namespace, output='screen')
    cameras = Node(package='hei_sensors', executable='cameras', namespace=namespace, output='screen',
                   parameters=[{'backend': LaunchConfiguration('camera_backend'), 'fps': 10.0}])
    monitor = Node(package='hei_monitor', executable='monitor', namespace=namespace, output='screen',
                   condition=IfCondition(LaunchConfiguration('monitor')),
                   parameters=[{'host': LaunchConfiguration('monitor_host'),
                                'port': int(LaunchConfiguration('monitor_port').perform(context))}])
    rviz = Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', str(description / 'rviz/hei.rviz')], output='screen')

    def after_states(event, _):
        return [motion] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='state broadcaster spawn failed'))]

    def after_motion(event, _):
        return [] if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='motion controller configuration failed'))]

    actions = [RegisterEventHandler(OnProcessExit(target_action=states, on_exit=after_states)),
               RegisterEventHandler(OnProcessExit(target_action=motion, on_exit=after_motion))]
    for critical in (control, rsp, manager):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=critical,
                       on_exit=[EmitEvent(event=Shutdown(reason='critical robot service exited'))])))
    return actions + [rsp, control, states, manager, cameras, monitor, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='hei'),
        DeclareLaunchArgument('camera_backend', default_value='mock', choices=['mock', 'opencv']),
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('monitor_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('monitor_port', default_value='8081'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
