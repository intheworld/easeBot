"""Real controller_manager/plugin over a PTY adapter; never opens motor devices."""
import errno
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces, SwitchController
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[1] / 'src'


class UsbCanAdapter:
    """Emulate the configured six motors and record every host frame."""
    def __init__(self, motors):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        os.close(slave)
        os.set_blocking(self.master, False)
        self.motors = {motor['motor_id']: motor for motor in motors}
        self.positions = {i: -0.5 if i in (2, 3) else 0.0 for i in self.motors}
        self.enabled = {i: False for i in self.motors}
        self.torques = {i: 0.0 for i in self.motors}
        self.frames = []
        self.error = None
        self.stopped = threading.Event()
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def reply(self, motor, payload):
        packet = b'\xaa\x11\x08' + struct.pack('<I', motor['feedback_id']) + bytes(payload) + b'\x55'
        os.write(self.master, packet)

    def status(self, motor_id):
        motor = self.motors[motor_id]
        def encode(value, limit, bits):
            return int((value + limit) / (2 * limit) * ((1 << bits) - 1))
        p = encode(self.positions[motor_id], motor['p_max'], 16)
        v = encode(0, motor['v_max'], 12)
        t = encode(self.torques[motor_id], motor['t_max'], 12)
        self.reply(motor, [motor_id | (0x10 if self.enabled[motor_id] else 0),
                          p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (t >> 8), t & 255, 30, 31])

    def handle(self, can_id, data):
        with self.lock:
            self.frames.append((can_id, data))
        if can_id == 0x7ff:
            motor_id = int.from_bytes(data[:2], 'little')
            if motor_id not in self.motors:
                raise AssertionError(f'unconfigured motor: {motor_id}')
            if data[2] == 0xcc:
                self.status(motor_id)
            elif data[2:4] == b'\x33\x0a':
                self.reply(self.motors[motor_id], data[:4] + struct.pack('<I', 2))
            else:
                raise AssertionError(f'unexpected parameter operation: {data.hex()}')
        elif can_id in self.motors and data[:7] == b'\xff' * 7:
            assert data[7] in (0xfc, 0xfd), 'driver must not zero or clear firmware faults'
            self.enabled[can_id] = data[7] == 0xfc
            self.status(can_id)
        elif can_id - 0x100 in self.motors:
            motor_id = can_id - 0x100
            target, speed = struct.unpack('<ff', data)
            assert 0 < speed <= 0.10001
            # Deliberately ideal motor following; this tests transport/controller
            # integration, not mechanical dynamics or actual stopping distance.
            self.positions[motor_id] = target
            self.status(motor_id)
        else:
            raise AssertionError(f'unexpected CAN ID: {can_id:x}')

    def run(self):
        buffer = bytearray()
        try:
            while not self.stopped.is_set():
                if not select.select([self.master], [], [], .01)[0]:
                    continue
                try:
                    block = os.read(self.master, 4096)
                except OSError as error:
                    if error.errno in (errno.EIO, errno.EAGAIN):
                        self.stopped.wait(.005)
                        continue
                    raise
                buffer.extend(block)
                while len(buffer) >= 30:
                    assert buffer[:3] == b'\x55\xaa\x1e', 'corrupt TX packet'
                    can_id = int.from_bytes(buffer[13:17], 'little')
                    self.handle(can_id, bytes(buffer[21:29]))
                    del buffer[:30]
        except BaseException as error:
            self.error = error

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)
        os.close(self.master)
        assert not self.thread.is_alive()


@pytest.mark.parametrize('readonly', [True, False])
def test_damiao_plugin_controller_and_readonly_boundary(tmp_path, monkeypatch, readonly):
    config = yaml.safe_load((ROOT / 'hei_bringup/config/right_arm.yaml').read_text())
    adapter = UsbCanAdapter(config['motors'])
    config['port'] = adapter.path
    config['feedback_timeout_ms'] = 500
    profile = tmp_path / 'arm.yaml'
    profile.write_text(yaml.safe_dump(config))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(120 + os.getpid() % 50))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros-log'))
    namespace = f'hei_damiao_test_{os.getpid()}_{int(readonly)}'
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ['ROS_DOMAIN_ID']))
    node = rclpy.create_node('damiao_acceptance', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    messages = []
    node.create_subscription(JointState, f'/{namespace}/joint_states', messages.append, qos_profile_sensor_data)
    log_path = tmp_path / 'bringup.log'
    launch = None
    try:
        with log_path.open('w+') as log:
            launch = subprocess.Popen(
                ['ros2', 'launch', 'hei_bringup', 'damiao_arm.launch.py',
                 'include_gripper:=false', f'config_file:={profile}', f'read_only:={str(readonly).lower()}', f'namespace:={namespace}'],
                stdout=log, stderr=log, env=env, start_new_session=True)

            def until(condition, timeout=20):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    assert adapter.error is None, repr(adapter.error)
                    assert launch.poll() is None, 'Damiao bringup exited'
                    executor.spin_once(timeout_sec=.05)
                    if condition():
                        return
                raise AssertionError('timed out waiting for Damiao ROS state')

            def call(client, request):
                until(client.service_is_ready)
                future = client.call_async(request)
                until(future.done)
                return future.result()

            listing = node.create_client(ListControllers, f'/{namespace}/controller_manager/list_controllers')
            hardware = node.create_client(ListHardwareInterfaces, f'/{namespace}/controller_manager/list_hardware_interfaces')
            until(lambda: messages)
            expected = {f'a_right_joint{i}' for i in range(1, 7)}
            assert set(messages[-1].name) == expected
            assert len(messages[-1].position) == 6
            interfaces = call(hardware, ListHardwareInterfaces.Request())
            assert len(interfaces.state_interfaces) == 18
            assert len(interfaces.command_interfaces) == (0 if readonly else 6)

            def controllers():
                return {c.name: c.state for c in call(listing, ListControllers.Request()).controller}

            deadline = time.monotonic() + 15
            current = controllers()
            while (current.get('joint_state_broadcaster') != 'active' or
                   (not readonly and current.get('right_arm_controller') != 'inactive')) and time.monotonic() < deadline:
                current = controllers()
            assert current['joint_state_broadcaster'] == 'active'
            if readonly:
                assert 'right_arm_controller' not in current
            else:
                assert current['right_arm_controller'] == 'inactive'
            assert not any(can_id != 0x7ff for can_id, _ in adapter.frames), 'startup sent motor commands'
            if not readonly:
                switching = node.create_client(SwitchController, f'/{namespace}/controller_manager/switch_controller')
                request = SwitchController.Request()
                request.activate_controllers = ['right_arm_controller']
                request.strictness = SwitchController.Request.STRICT
                request.timeout.sec = 3
                assert call(switching, request).ok
                until(lambda: all(adapter.enabled.values()))
                action = ActionClient(node, FollowJointTrajectory, f'/{namespace}/right_arm_controller/follow_joint_trajectory')
                until(action.server_is_ready)
                goal = FollowJointTrajectory.Goal()
                goal.trajectory.joint_names = [f'a_right_joint{i}' for i in range(1, 7)]
                point = JointTrajectoryPoint(positions=[0.02, -0.52, -0.52, 0.02, 0.02, 0.02])
                point.time_from_start.sec = 1
                goal.trajectory.points = [point]
                future = action.send_goal_async(goal)
                until(future.done)
                handle = future.result()
                assert handle.accepted
                result = handle.get_result_async()
                until(result.done)
                assert result.result().result.error_code == 0
                request = SwitchController.Request()
                request.deactivate_controllers = ['right_arm_controller']
                request.strictness = SwitchController.Request.STRICT
                request.timeout.sec = 3
                assert call(switching, request).ok
                until(lambda: not any(adapter.enabled.values()))
    except BaseException:
        if log_path.exists():
            print(log_path.read_text()[-16000:])
        raise
    finally:
        if launch is not None and launch.poll() is None:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait(timeout=3)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        adapter.close()
    assert adapter.error is None, repr(adapter.error)
    if readonly:
        assert adapter.frames
        assert all(can_id == 0x7ff and data[2] == 0xcc for can_id, data in adapter.frames)


class GripperProbeAdapter(UsbCanAdapter):
    def __init__(self, motors):
        self.mode = 1
        super().__init__(motors)
        self.positions[7] = -0.001

    def handle(self, can_id, data):
        if can_id == 0x7ff and data[2] == 0x55:
            with self.lock:
                self.frames.append((can_id, data))
            assert data[:4] == b'\x07\x00\x55\x0a'
            self.mode = int.from_bytes(data[4:], 'little')
            assert self.mode in (1, 4)
            self.reply(self.motors[7], data)
        elif can_id == 0x7ff and data[2:4] == b'\x33\x0a':
            with self.lock:
                self.frames.append((can_id, data))
            assert int.from_bytes(data[:2], 'little') == 7
            self.reply(self.motors[7], data[:4] + struct.pack('<I', self.mode))
        elif can_id == 0x307:
            with self.lock:
                self.frames.append((can_id, data))
            position, speed, current = struct.unpack('<fHH', data)
            assert -4.5 <= position <= 0
            assert speed == 5 and current == 100  # 0.05 rad/s, 1% current
            assert self.mode == 4
            if self.enabled[7]:
                self.positions[7] = position
            self.status(7)
        else:
            if can_id != 0x7ff:
                assert can_id == 7 and data[:7] == b'\xff' * 7, 'no arm motor commands permitted'
            super().handle(can_id, data)


@pytest.mark.parametrize('move', [False, True])
def test_gripper_probe_readonly_and_motor7_only_bounded_motion(move):
    binary = ROOT.parents[0] / 'build/hei_hardware/motorbridge-target/release/examples/gripper_probe'
    if not binary.is_file():
        pytest.skip('build motorbridge_backend --example gripper_probe first')
    config = yaml.safe_load((ROOT / 'hei_bringup/config/right_arm.yaml').read_text())
    gripper = dict(config['motors'][-1], name='gripper_motor', motor_id=7, feedback_id=23)
    adapter = GripperProbeAdapter(config['motors'] + [gripper])
    try:
        command = [str(binary), '--port', adapter.path]
        if move:
            command.append('--move')
        result = subprocess.run(command, text=True, capture_output=True, timeout=10)
        assert adapter.error is None, repr(adapter.error)
        assert result.returncode == 0, result.stdout + result.stderr
        assert not any(adapter.enabled.values())
        frames = list(adapter.frames)
        if move:
            assert 'reached=true' in result.stdout
            assert adapter.mode == 1
            assert -0.023 < adapter.positions[7] < -0.019
            assert any(i == 7 and d[-1] == 0xfc for i, d in frames)
            assert any(i == 7 and d[-1] == 0xfd for i, d in frames)
            assert any(i == 0x307 for i, _ in frames)
        else:
            assert adapter.mode == 1
            assert all(i == 0x7ff and d[2] in (0xcc, 0x33) for i, d in frames)
    finally:
        adapter.close()


def test_gripper_probe_sigterm_disables_only_gripper_and_restores_mode():
    binary = ROOT.parents[0] / 'build/hei_hardware/motorbridge-target/release/examples/gripper_probe'
    if not binary.is_file():
        pytest.skip('build motorbridge_backend --example gripper_probe first')
    config = yaml.safe_load((ROOT / 'hei_bringup/config/right_arm.yaml').read_text())
    gripper = dict(config['motors'][-1], name='gripper_motor', motor_id=7, feedback_id=23)
    adapter = GripperProbeAdapter(config['motors'] + [gripper])
    process = None
    try:
        process = subprocess.Popen([str(binary), '--port', adapter.path, '--move'],
                                   text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not adapter.enabled[7]:
            assert process.poll() is None
            time.sleep(.01)
        assert adapter.enabled[7], 'gripper was not enabled'
        process.send_signal(signal.SIGTERM)
        out, error = process.communicate(timeout=3)
        assert process.returncode != 0
        assert 'interrupted' in error, out + error
        assert adapter.error is None, repr(adapter.error)
        assert not any(adapter.enabled.values())
        assert adapter.mode == 1
    finally:
        if process and process.poll() is None:
            process.kill()
            process.wait(timeout=2)
        adapter.close()
