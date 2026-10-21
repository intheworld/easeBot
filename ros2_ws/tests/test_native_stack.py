"""End-to-end test against real Jazzy nodes/controllers, using only mock hardware."""
import http.client
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, SwitchController
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from sensor_msgs.msg import JointState, CompressedImage, Image
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from trajectory_msgs.msg import JointTrajectoryPoint


def test_native_launch_controls_topics_stop_and_readonly_monitor():
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    domain = str(80 + os.getpid() % 100)
    env = {**os.environ, 'ROS_DOMAIN_ID': domain, 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'}
    env.pop('ROS_LOCALHOST_ONLY', None)
    old_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    with tempfile.TemporaryDirectory(prefix='hei-native-') as temp:
        env['ROS_LOG_DIR'] = temp
        with open(Path(temp) / 'launch.log', 'w+') as log:
            launch = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'robot.launch.py',
                                       f'monitor_port:={port}'], env=env, stdout=log, stderr=log, start_new_session=True)
            node = None
            rclpy.init()
            try:
                node = rclpy.create_node('native_acceptance_test')
                states, images, transforms = [], [], []
                node.create_subscription(JointState, '/hei/joint_states', states.append, qos_profile_sensor_data)
                node.create_subscription(CompressedImage, '/hei/cameras/front/image_raw/compressed', images.append, qos_profile_sensor_data)
                node.create_subscription(TFMessage, '/tf', transforms.append, 10)
                camera_images = {'front': images, 'left_wrist': [], 'right_wrist': []}
                for name in ('left_wrist', 'right_wrist'):
                    node.create_subscription(CompressedImage,
                        f'/hei/cameras/{name}/image_raw/compressed', camera_images[name].append,
                        qos_profile_sensor_data)
                raw_images = []
                node.create_subscription(Image, '/hei/cameras/front/image_raw', raw_images.append,
                                         qos_profile_sensor_data)
                listing = node.create_client(ListControllers, '/hei/controller_manager/list_controllers')

                def spin_until(condition, timeout=10):
                    end = time.monotonic() + timeout
                    while time.monotonic() < end:
                        assert launch.poll() is None, 'launch exited unexpectedly'
                        rclpy.spin_once(node, timeout_sec=0.05)
                        if condition():
                            return
                    log.flush()
                    raise AssertionError('timed out waiting for ROS state')

                def call(client, request):
                    assert client.wait_for_service(timeout_sec=10)
                    future = client.call_async(request)
                    spin_until(future.done, 10)
                    return future.result()

                def controllers():
                    return {c.name: c.state for c in call(listing, ListControllers.Request()).controller}

                assert listing.wait_for_service(timeout_sec=20)
                deadline = time.monotonic() + 25
                current = controllers()
                while (len(current) < 7 or any(s != 'inactive' for n,s in current.items() if n != 'joint_state_broadcaster')) and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    current = controllers()
                assert current['joint_state_broadcaster'] == 'active'
                assert len(current) == 7
                assert all(s == 'inactive' for n, s in current.items() if n != 'joint_state_broadcaster')
                spin_until(lambda: states and images and transforms)
                spin_until(lambda: ('monitor', '/hei') in node.get_node_names_and_namespaces())
                expected_monitor_topics = {'/hei/joint_states', '/hei/diagnostics', '/hei/base_controller/odom'}
                expected_monitor_topics |= {f'/hei/cameras/{name}/image_raw/compressed' for name in ('front', 'left_wrist', 'right_wrist')}
                spin_until(lambda: expected_monitor_topics.issubset({name for name, _ in node.get_subscriber_names_and_types_by_node('monitor', '/hei')}))
                assert node.get_client_names_and_types_by_node('monitor', '/hei') == []
                publishers = {name for name, _ in node.get_publisher_names_and_types_by_node('monitor', '/hei')}
                assert publishers.issubset({'/rosout', '/parameter_events'})
                assert 'lift_joint' in states[-1].name and len(states[-1].name) == 21
                assert images[-1].header.frame_id == 'front_camera_optical_frame'
                assert bytes(images[-1].data).startswith(b'\xff\xd8')
                assert any(t.child_frame_id == 'a_right_link1' for msg in transforms for t in msg.transforms)
                # Bringup autostarts the LifecycleNode once; manual deactivate must persist.
                camera_state = node.create_client(GetState, '/hei/cameras/get_state')
                camera_change = node.create_client(ChangeState, '/hei/cameras/change_state')
                assert call(camera_state, GetState.Request()).current_state.id == State.PRIMARY_STATE_ACTIVE
                spin_until(lambda: all(camera_images.values()) and raw_images)

                def camera_transition(transition, expected):
                    request = ChangeState.Request()
                    request.transition.id = transition
                    assert call(camera_change, request).success
                    assert call(camera_state, GetState.Request()).current_state.id == expected

                camera_transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
                drain_until = time.monotonic() + 0.3
                spin_until(lambda: time.monotonic() >= drain_until)
                counts = {name: len(frames) for name, frames in camera_images.items()}
                raw_count = len(raw_images)
                pause_until = time.monotonic() + 1.3
                spin_until(lambda: time.monotonic() >= pause_until)
                assert {name: len(frames) for name, frames in camera_images.items()} == counts
                assert len(raw_images) == raw_count
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/v1/status', timeout=2) as response:
                    paused = json.load(response)
                assert all(not c['valid'] for c in paused['sensors']['data']['cameras'].values())
                camera_transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)
                spin_until(lambda: all(len(camera_images[name]) > count for name, count in counts.items())
                           and len(raw_images) > raw_count)
                # Also exercise resource destruction and configure again in the same process.
                camera_transition(Transition.TRANSITION_DEACTIVATE, State.PRIMARY_STATE_INACTIVE)
                camera_transition(Transition.TRANSITION_CLEANUP, State.PRIMARY_STATE_UNCONFIGURED)
                camera_transition(Transition.TRANSITION_CONFIGURE, State.PRIMARY_STATE_INACTIVE)
                camera_transition(Transition.TRANSITION_ACTIVATE, State.PRIMARY_STATE_ACTIVE)
                camera_images['front'].clear()
                spin_until(lambda: camera_images['front'])
                action = ActionClient(node, FollowJointTrajectory, '/hei/right_arm_controller/follow_joint_trajectory')
                assert action.wait_for_server(timeout_sec=10)
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = [f'a_right_joint{i}' for i in range(1,7)]
                goal.trajectory.points = [JointTrajectoryPoint(positions=[0.1,-0.5,-0.5,0.,0.,0.],
                                                               time_from_start=rclpy.duration.Duration(seconds=1).to_msg())]
                request = action.send_goal_async(goal)
                spin_until(request.done)
                assert not request.result().accepted, 'inactive controller must reject motion'
                enabled = node.create_client(SetBool, '/hei/robot_manager/set_enabled')
                assert call(enabled, SetBool.Request(data=True)).success
                request = action.send_goal_async(goal)
                spin_until(request.done)
                assert request.result().accepted
                result = request.result().get_result_async()
                spin_until(result.done)
                assert result.result().result.error_code == 0
                spin_until(lambda: abs(dict(zip(states[-1].name, states[-1].position))['a_right_joint1']-0.1)<0.01)
                # Verify prismatic SI units and the passive gripper mimic state as well.
                def move_group(group, names, positions):
                    client = ActionClient(node, FollowJointTrajectory, '/hei/' + group + '_controller/follow_joint_trajectory')
                    assert client.wait_for_server(timeout_sec=5)
                    command = FollowJointTrajectory.Goal()
                    command.trajectory.joint_names = names
                    command.trajectory.points = [JointTrajectoryPoint(positions=positions,
                        time_from_start=rclpy.duration.Duration(seconds=1).to_msg())]
                    sent = client.send_goal_async(command)
                    spin_until(sent.done)
                    assert sent.result().accepted
                    completed = sent.result().get_result_async()
                    spin_until(completed.done)
                    assert completed.result().result.error_code == 0
                    client.destroy()
                move_group('lift', ['lift_joint'], [-0.21])
                move_group('right_gripper', ['a_right_end_joint_L_finger'], [0.01])
                spin_until(lambda: abs(dict(zip(states[-1].name, states[-1].position))['a_right_end_joint_R_finger'] + 0.01) < 0.001)
                # Standard omni controller times out stale velocity inputs on its own update loop.
                velocity = node.create_publisher(TwistStamped, '/hei/base_controller/cmd_vel', 10)
                spin_until(lambda: velocity.get_subscription_count() > 0)
                for _ in range(4):
                    msg = TwistStamped()
                    msg.header.stamp = node.get_clock().now().to_msg()
                    msg.twist.linear.x = 0.05
                    velocity.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.05)
                wheels = [n for n in states[-1].name if 'wheel_joint' in n]
                spin_until(lambda: any(abs(dict(zip(states[-1].name, states[-1].velocity))[n]) > 0.01 for n in wheels))
                begin = time.monotonic()
                spin_until(lambda: time.monotonic()-begin > 0.4 and all(abs(dict(zip(states[-1].name, states[-1].velocity))[n]) < 1e-6 for n in wheels), 3)
                stop = node.create_client(Trigger, '/hei/robot_manager/stop')
                assert call(stop, Trigger.Request()).success
                assert all(s == 'inactive' for n,s in controllers().items() if n != 'joint_state_broadcaster')
                # Verify actual HTTP behavior, not just a frontend lacking buttons.
                for method in ('POST','PUT','PATCH','DELETE'):
                    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
                    conn.request(method, '/api/v1/control/enable', '{}')
                    response = conn.getresponse()
                    assert response.status == 405
                    response.read()
                    conn.close()
                deadline = time.monotonic() + 3
                while True:
                    with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/v1/status', timeout=2) as response:
                        data = json.load(response)
                    if data['robot']['data']['mode'] == 'DISABLED':
                        break
                    assert time.monotonic() < deadline
                    rclpy.spin_once(node, timeout_sec=0.1)
                assert data['system']['monitor_read_only']
                assert 'Rust' in data['system']['version']
                assert data['system']['memory']['MemTotal'] > 0
                for path, mime in (('/', 'text/html'), ('/app.js', 'text/javascript'), ('/style.css', 'text/css'), ('/api/v1/cameras/front', 'image/jpeg')):
                    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}', timeout=2) as response:
                        assert response.headers['Content-Type'].startswith(mime)
                        assert response.headers['Cache-Control'] == 'no-store'
                        assert response.read()
                assert data['robot']['data']['backend'] == 'ros2_control_mock'
                assert data['robot']['data']['valid']
                assert all(c['valid'] for c in data['sensors']['data']['cameras'].values())
                assert controllers()['right_arm_controller'] == 'inactive'
                # Losing feedback must request deactivation even while ROS services still work.
                assert call(enabled, SetBool.Request(data=True)).success
                switching = node.create_client(SwitchController, '/hei/controller_manager/switch_controller')
                req = SwitchController.Request()
                req.deactivate_controllers = ['joint_state_broadcaster']
                req.strictness = SwitchController.Request.STRICT
                req.timeout.sec = 2
                assert call(switching, req).ok
                deadline = time.monotonic() + 5
                while any(v == 'active' for k,v in controllers().items() if k != 'joint_state_broadcaster'):
                    assert time.monotonic() < deadline, 'manager failed to request stop on stale feedback'
                    rclpy.spin_once(node, timeout_sec=0.1)
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/v1/status', timeout=2) as response:
                    stale = json.load(response)
                assert not stale['robot']['data']['valid']
            finally:
                failed = sys.exc_info()[0] is not None
                if node is not None:
                    node.destroy_node()
                rclpy.shutdown()
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
                if failed or launch.returncode:
                    log.seek(0)
                    print(log.read()[-12000:])
                if old_domain is None:
                    os.environ.pop('ROS_DOMAIN_ID', None)
                else:
                    os.environ['ROS_DOMAIN_ID'] = old_domain
