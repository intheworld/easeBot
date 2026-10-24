"""Unified real-hardware launch with four PTY motor buses and passive limit IO."""
import os
import json
import socket
import signal
import struct
import subprocess
import time
import uuid
import urllib.request
import urllib.error

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectoryPoint
from test_base_stack import BaseAdapter
from test_damiao_stack import ROOT
from test_lift_stack import LiftMotor, LimitIO
from test_shared_gripper import SharedAdapter


@pytest.mark.parametrize('case', ['default', 'readonly', 'duplicate', 'io_failure', 'timeout'])
def test_whole_robot_defaults_readonly_and_startup_failure(tmp_path, monkeypatch, case):
    domain = 150 + os.getpid() % 30
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    adapters, profiles = {}, {}
    for side in ('left', 'right'):
        config = yaml.safe_load((ROOT / f'hei_bringup/config/{side}_arm.yaml').read_text())
        adapters[side] = SharedAdapter(config['motors'] + [config['gripper']])
        config.update(port=adapters[side].path, feedback_timeout_ms=1000)
        profiles[side] = config
    config = yaml.safe_load((ROOT / 'hei_bringup/config/base.yaml').read_text())
    adapters['base'] = BaseAdapter(config['motors'])
    adapters['base'].modes = {i: 1 for i in adapters['base'].motors}
    config.update(port=adapters['base'].path, feedback_timeout_ms=1000)
    profiles['base'] = config
    motor = adapters['lift'] = LiftMotor()
    io = LimitIO(motor, automatic=True, corrupt=case == 'io_failure')
    config = yaml.safe_load((ROOT / 'hei_bringup/config/lift.yaml').read_text())
    config.update(port=motor.path, io_port=io.path, feedback_timeout_ms=1000,
                  home_timeout_sec=5., io_wait_timeout_sec=.6)
    if case == 'duplicate':
        config['io_port'] = adapters['base'].path
    profiles['lift'] = config
    args = ['camera_backend:=mock']  # Keep camera input virtual, like the PTY motors.
    for component, config in profiles.items():
        path = tmp_path / f'{component}.yaml'
        path.write_text(yaml.safe_dump(config))
        args.append(f'{component}_config_file:={path}')
    if case == 'readonly':
        args.append('read_only:=true')
    if case == 'timeout':
        args.append('startup_timeout_sec:=0.01')
    ns = 'whole_' + uuid.uuid4().hex[:8]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        monitor_port = sock.getsockname()[1]
    args.append(f'monitor_port:={monitor_port}')
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('whole_robot_acceptance', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log_path = tmp_path / 'robot.log'
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'robot.launch.py',
        f'namespace:={ns}', *args], stdout=log, stderr=log,
        env=dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'roslog')), start_new_session=True)

    def until(predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()[-22000:]
            assert all(a.error is None for a in adapters.values()), [a.error for a in adapters.values()]
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(log_path.read_text()[-22000:])

    def wait(future):
        until(future.done)
        return future.result()

    def payload():
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{monitor_port}/api/v1/status', timeout=1) as response:
                return json.load(response)
        except (OSError, urllib.error.URLError):
            return {}

    def status():
        return payload().get('robot', {}).get('data', {})

    def trajectory(controller, names, positions):
        client = ActionClient(node, FollowJointTrajectory, f'/{ns}/{controller}/follow_joint_trajectory')
        until(client.server_is_ready)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint(positions=positions)
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        return wait(client.send_goal_async(goal))

    try:
        if case in ('duplicate', 'io_failure', 'timeout'):
            process.wait(timeout=25)
            text = log_path.read_text()
            assert 'Robot actuators ready:' not in text
            assert {'duplicate': 'serial device conflict', 'io_failure': 'no valid lift IO frame',
                    'timeout': 'Robot actuator startup timed out'}[case] in text
            if case == 'duplicate':
                assert all(not a.frames for a in adapters.values())
        else:
            until(lambda: 'Robot actuators ready:' in log_path.read_text(), timeout=90)
            expected_mode = 'READ_ONLY' if case == 'readonly' else 'READY'
            # The HTTP observer polls independently of the launch readiness
            # helper; allow its next controller query to see the same state.
            until(lambda: status().get('mode') == expected_mode and status().get('valid') is True)
            monitored = status()
            assert monitored['backend'] == 'ros2_control_damiao'
            assert monitored['hardware_connected']
            assert set(monitored['buses']) == {'left', 'right', 'base', 'lift'}
            assert monitored['enabled'] is None  # Active controller is not motor enable feedback.
            assert monitored['homed'] == (case != 'readonly')
            assert monitored['mode'] == ('READ_ONLY' if case == 'readonly' else 'READY')
            assert len(monitored['state']) == 19
            def cameras_online():
                cameras = payload().get('sensors', {}).get('data', {}).get('cameras', {})
                return len(cameras) == 3 and all(c['valid'] and c['backend'] == 'mock' for c in cameras.values())
            until(cameras_online)
            for camera in ('front', 'left_wrist', 'right_wrist'):
                with urllib.request.urlopen(f'http://127.0.0.1:{monitor_port}/api/v1/cameras/{camera}', timeout=2) as image:
                    assert image.headers['Content-Type'] == 'image/jpeg'
                    assert image.read().startswith(b'\xff\xd8')
            request = urllib.request.Request(f'http://127.0.0.1:{monitor_port}/api/v1/status', method='POST')
            with pytest.raises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(request, timeout=1)
            assert error.value.code == 405
            for component in adapters:
                client = node.create_client(ListControllers, f'/{ns}/{component}_controller_manager/list_controllers')
                until(client.service_is_ready)
                states = {c.name: c.state for c in wait(client.call_async(ListControllers.Request())).controller}
                expected = {f'{component}_joint_state_broadcaster': 'active'}
                if component in ('left', 'right'):
                    expected[f'{component}_gripper_controller'] = 'inactive' if case == 'readonly' else 'active'
                    if case != 'readonly':
                        expected[f'{component}_arm_controller'] = 'active'
                elif case != 'readonly':
                    expected[f'{component}_controller'] = 'active'
                assert states == expected
            if case == 'readonly':
                assert all(can_id == 0x7ff and data[2] == 0xcc
                           for a in adapters.values() for can_id, data in a.frames)
            else:
                assert set(adapters['base'].modes.values()) == {3}
                assert any(speed < 0 for _, speed in motor.speeds)  # Default lift homing ran.
                for side, prefix in [('left', 'b_left'), ('right', 'a_right')]:
                    arm = trajectory(f'{side}_arm_controller', [f'{prefix}_joint{i}' for i in range(1, 7)],
                                     [0., -.49, -.5, 0., 0., 0.])
                    assert arm.accepted
                    client = ActionClient(node, GripperCommand, f'/{ns}/{side}_gripper_controller/gripper_cmd')
                    until(client.server_is_ready)
                    goal = GripperCommand.Goal()
                    goal.command.position = .002
                    gripper = wait(client.send_goal_async(goal))
                    assert gripper.accepted
                    assert wait(gripper.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
                    assert wait(arm.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
                assert not trajectory('lift_controller', ['lift_joint'], [.05]).accepted
                lift = trajectory('lift_controller', ['lift_joint'], [-.004])
                assert lift.accepted
                assert wait(lift.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
                pub = node.create_publisher(TwistStamped, f'/{ns}/base_controller/cmd_vel', 10)
                until(lambda: pub.get_subscription_count() > 0)
                message = TwistStamped()
                message.header.stamp = node.get_clock().now().to_msg()
                message.twist.linear.x = .01
                pub.publish(message)
                until(lambda: len({cid for cid, data in adapters['base'].frames
                    if cid - 0x200 in adapters['base'].motors and abs(struct.unpack('<f', data[:4])[0]) > .01}) == 4)
                # Keep accepting stop frames, but simulate loss of all wheel feedback.
                adapters['base'].status = lambda _: None
                until(lambda: status().get('buses', {}).get('base', {}).get('connected') is False)
                monitored = status()
                assert not monitored['hardware_connected'] and not monitored['valid']
                assert monitored['buses']['left']['valid'] and monitored['buses']['right']['valid']
                assert monitored['state']['front_left_wheel_joint'] is None
                assert monitored['state']['a_right_joint1'] is not None
    except BaseException:
        print(log_path.read_text()[-22000:])
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        for adapter in adapters.values():
            adapter.close()
        io.close()
    assert all(not any(a.enabled.values()) for a in adapters.values())
    assert all(a.error is None for a in adapters.values())
