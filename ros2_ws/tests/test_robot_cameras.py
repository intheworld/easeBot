"""Opt-in physical camera acceptance test; never opens motor devices."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('HEI_TEST_REAL_CAMERAS') != '1',
                                reason='set HEI_TEST_REAL_CAMERAS=1 on the robot to open its cameras')


def test_physical_camera_lifecycle(tmp_path, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    import cv2
    import numpy as np
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    names = ('front', 'left_wrist', 'right_wrist')
    for name in names:
        assert Path(f'/dev/hei_{name}_camera').exists(), f'missing {name} device'
    monkeypatch.setenv('ROS_DOMAIN_ID', str(150 + os.getpid() % 30))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    namespace = 'hei_camera_acceptance'
    counts = {name: 0 for name in names}
    stamps = {name: set() for name in names}
    sizes, diagnostics = {}, []
    context = rclpy.context.Context()
    node = None
    executor = None
    process = None
    log_path = tmp_path / 'cameras.log'
    with log_path.open('w+') as log:
        try:
            process = subprocess.Popen(
                ['ros2', 'run', 'hei_sensors', 'cameras', '--ros-args', '-r', f'__ns:=/{namespace}',
                 '--params-file', str(Path(__file__).resolve().parents[2] / 'deploy/robot/cameras.yaml')],
                env=env, stdout=log, stderr=log, start_new_session=True)
            rclpy.init(context=context)
            node = rclpy.create_node('physical_camera_acceptance', context=context)
            executor = rclpy.executors.SingleThreadedExecutor(context=context)
            executor.add_node(node)

            def receive(name, message):
                decoded = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
                assert decoded is not None, f'{name}: invalid JPEG'
                counts[name] += 1
                stamps[name].add((message.header.stamp.sec, message.header.stamp.nanosec))
                sizes[name] = list(decoded.shape[:2][::-1])

            for name in names:
                node.create_subscription(CompressedImage, f'/{namespace}/cameras/{name}/image_raw/compressed',
                                         lambda message, name=name: receive(name, message), qos_profile_sensor_data)
            node.create_subscription(DiagnosticArray, f'/{namespace}/diagnostics', diagnostics.append, 10)
            change = node.create_client(ChangeState, f'/{namespace}/cameras/change_state')

            def spin_until(condition, timeout=15):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, 'camera process exited'
                    executor.spin_once(timeout_sec=.05)
                    if condition():
                        return
                raise AssertionError('timed out waiting for camera data/transition')

            def transition(identifier):
                request = ChangeState.Request()
                request.transition.id = identifier
                future = change.call_async(request)
                spin_until(future.done)
                assert future.result().success, f'failed transition {identifier}'

            assert change.wait_for_service(timeout_sec=15)
            transition(Transition.TRANSITION_CONFIGURE)
            assert not any(counts.values())
            transition(Transition.TRANSITION_ACTIVATE)
            spin_until(lambda: all(len(stamps[name]) >= 3 for name in names))
            spin_until(lambda: any(len(msg.status) == 3 and all(status.level == DiagnosticStatus.OK for status in msg.status)
                                   for msg in diagnostics))
            transition(Transition.TRANSITION_DEACTIVATE)
            # Drain in-flight sensor packets before checking inactive output.
            end = time.monotonic() + .4
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.05)
            inactive_counts, diag_count = dict(counts), len(diagnostics)
            end = time.monotonic() + 1.2
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.05)
            assert counts == inactive_counts, 'image publication continued while inactive'
            assert len(diagnostics) > diag_count, 'diagnostics stopped while inactive'
            transition(Transition.TRANSITION_ACTIVATE)
            spin_until(lambda: all(counts[name] > inactive_counts[name] for name in names))
            transition(Transition.TRANSITION_DEACTIVATE)
            transition(Transition.TRANSITION_CLEANUP)
            # Reopen verifies cleanup released the actual camera handles.
            transition(Transition.TRANSITION_CONFIGURE)
            transition(Transition.TRANSITION_CLEANUP)
            print('Physical camera acceptance:', json.dumps({'frames': counts, 'sizes': sizes,
                                                            'deactivate_reactivate_cleanup': 'passed'}))
        finally:
            if process is not None:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
            if executor is not None:
                executor.shutdown(timeout_sec=2)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()
            log.seek(0)
            print('Observed frames:', counts, 'sizes:', sizes)
            print('Latest diagnostics:', [[(s.name, s.level, s.message, [(v.key, v.value) for v in s.values])
                                           for s in msg.status] for msg in diagnostics[-2:]])
            print(log.read())
