"""Two PTYs emulate the lift motor and passive limit IO. Never real devices."""
import errno
import os
import signal
import struct
import subprocess
import threading
import time
import uuid

import pytest
import yaml
rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces, SwitchController
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
from test_damiao_stack import ROOT, UsbCanAdapter


def io_frame(bits):
    data = bytes([1, 4, 2, 0, bits])
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return data + crc.to_bytes(2, 'little')


class LiftMotor(UsbCanAdapter):
    def __init__(self, initial=0.0):
        self.absolute = initial
        self.speed = 0.0
        self.last_status = time.monotonic()
        self.speeds = []
        self.raw_history = []
        super().__init__([dict(motor_id=1, feedback_id=17, p_max=12.5, v_max=30.0, t_max=10.0)])
        self.modes[1] = 3

    def status(self, motor_id):
        now = time.monotonic()
        if self.enabled[motor_id]:
            self.absolute += self.speed * (now - self.last_status)
        self.last_status = now
        raw = (self.absolute + 12.5) % 25 - 12.5
        self.raw_history.append(raw)
        encode = lambda v, limit, bits: int((v + limit) / (2 * limit) * ((1 << bits) - 1))
        q = encode(raw, 12.5, 16)
        v = encode(self.speed if self.enabled[1] else 0, 30, 12)
        t = encode(0, 10, 12)
        self.reply(self.motors[1], [1 | (0x10 if self.enabled[1] else 0), q >> 8, q & 255,
            v >> 4, ((v & 15) << 4) | (t >> 8), t & 255, 30, 30])

    def handle(self, can_id, data):
        if can_id == 0x201:
            with self.lock:
                self.frames.append((can_id, data))
            self.status(1)  # Integrate the previous speed before accepting the new one.
            self.speed = struct.unpack('<f', data[:4])[0]
            assert abs(self.speed) <= 18.00001 and self.modes[1] == 3
            self.speeds.append((time.monotonic(), self.speed))
            return
        super().handle(can_id, data)


class LimitIO:
    def __init__(self, motor, upper=False, automatic=False, corrupt=False):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        os.close(slave)
        self.motor = motor
        self.bits = 1 if upper else 0
        self.automatic = automatic
        self.corrupt = corrupt
        self.paused = False
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.wait(.01):
            if self.paused:
                continue
            bits = (1 if self.motor.absolute <= -.15 else 0) if self.automatic else self.bits
            frame = io_frame(bits)
            if self.corrupt:
                frame = frame[:-1] + bytes([frame[-1] ^ 0xff])
            try:
                # Fragments exercise stream reassembly, not only complete packets.
                os.write(self.master, b'\xff\xaa' + frame[:3])
                os.write(self.master, frame[3:])
            except OSError as error:
                if error.errno != errno.EIO:
                    self.error = error
                    return

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        os.close(self.master)


@pytest.fixture
def lift(tmp_path, monkeypatch, request):
    settings = getattr(request, 'param', {})
    readonly = settings.get('readonly', False)
    home = settings.get('home', True)
    failure = settings.get('startup_failure', False)
    motor = LiftMotor(settings.get('initial', 0.0))
    io = LimitIO(motor, upper=settings.get('upper', False), automatic=settings.get('automatic', False),
                 corrupt=settings.get('corrupt', False))
    config = yaml.safe_load((ROOT / 'hei_bringup/config/lift.yaml').read_text())
    config.update(port=motor.path, io_port=io.path, feedback_timeout_ms=500,
                  io_timeout_sec=.3, io_wait_timeout_sec=.6, home_timeout_sec=settings.get('home_timeout', 4.0))
    profile = tmp_path / 'lift.yaml'
    profile.write_text(yaml.safe_dump(config))
    domain = 120 + os.getpid() % 30
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    ns = 'lift_test_' + uuid.uuid4().hex[:8]
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('lift_acceptance', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    status = {}

    def dynamic(message):
        for name, values in zip(message.joint_names, message.interface_values):
            status[name] = dict(zip(values.interface_names, values.values))

    node.create_subscription(DynamicJointState, f'/{ns}/lift_dynamic_joint_states', dynamic, qos_profile_sensor_data)
    log_path = tmp_path / 'lift.log'
    log = log_path.open('w')
    env = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'roslog'))
    if settings.get('poison_python'):
        # Simulate Conda's env python3 selecting an incompatible interpreter.
        wrong_python = tmp_path / 'bin/python3'
        wrong_python.parent.mkdir()
        wrong_python.write_text('#!/bin/sh\necho wrong-python-ABI >&2\nexit 91\n')
        wrong_python.chmod(0o755)
        env['PATH'] = str(wrong_python.parent) + os.pathsep + env['PATH']
    process = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'damiao_lift.launch.py',
        f'namespace:={ns}', f'config_file:={profile}', f'read_only:={str(readonly).lower()}',
        f'home:={str(home).lower()}', 'configure_control_mode:=true'],
        stdout=log, stderr=log, env=env, start_new_session=True)

    def until(predicate, timeout=20, allow_exit=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert allow_exit or process.poll() is None, log_path.read_text()[-18000:]
            assert motor.error is None and io.error is None, (motor.error, io.error)
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(log_path.read_text()[-18000:])

    def wait(future, timeout=12):
        until(future.done, timeout)
        return future.result()

    def call(client, req):
        until(client.service_is_ready)
        return wait(client.call_async(req))

    listing = node.create_client(ListControllers, f'/{ns}/lift_controller_manager/list_controllers')
    action = ActionClient(node, FollowJointTrajectory, f'/{ns}/lift_controller/follow_joint_trajectory')

    def controllers():
        return {c.name: c.state for c in call(listing, ListControllers.Request()).controller}

    def goal(height):
        until(action.server_is_ready)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ['lift_joint']
        g.trajectory.points = [JointTrajectoryPoint(positions=[height])]
        g.trajectory.points[0].time_from_start.sec = 1
        return wait(action.send_goal_async(g))

    try:
        if not failure:
            expected = {'lift_joint_state_broadcaster': 'active'}
            if not readonly:
                expected['lift_controller'] = 'active' if home else 'inactive'
            until(lambda: controllers() == expected)
            until(lambda: 'lift_status' in status)
        yield dict(motor=motor, io=io, status=status, until=until, wait=wait, call=call, action=action,
                   controllers=controllers, goal=goal, node=node, ns=ns, process=process, log=log_path)
    except BaseException:
        print(log_path.read_text()[-18000:])
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        action.destroy()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        motor.close()
        io.close()
    assert not any(motor.enabled.values())
    assert motor.error is None and io.error is None


@pytest.mark.parametrize('lift', [{'readonly': True}], indirect=True)
def test_lift_readonly_never_enables_writes_mode_or_zero(lift):
    m = lift['motor']
    assert not m.enabled[1]
    assert all(can_id == 0x7ff and data[2] == 0xcc for can_id, data in m.frames)
    assert lift['status']['lift_status']['homed'] == 0
    info = lift['node'].create_client(ListHardwareInterfaces, f"/{lift['ns']}/lift_controller_manager/list_hardware_interfaces")
    result = lift['call'](info, ListHardwareInterfaces.Request())
    assert len(result.state_interfaces) == 9 and not result.command_interfaces


@pytest.mark.parametrize('lift', [{'home': False}], indirect=True)
def test_lift_requires_home_before_height_controller_claim(lift):
    assert not lift['motor'].enabled[1]
    client = lift['node'].create_client(SwitchController, f"/{lift['ns']}/lift_controller_manager/switch_controller")
    req = SwitchController.Request()
    req.activate_controllers = ['lift_controller']
    req.strictness = SwitchController.Request.STRICT
    req.timeout.sec = 2
    assert not lift['call'](client, req).ok
    assert not lift['motor'].enabled[1]
    assert lift['controllers']()['lift_joint_state_broadcaster'] == 'active'


@pytest.mark.parametrize('lift', [{'automatic': True}], indirect=True)
def test_lift_homes_to_upper_without_firmware_zero_then_moves_in_meters(lift):
    assert lift['status']['lift_status']['homed'] == 1
    assert any(speed < -.1 for _, speed in lift['motor'].speeds)
    handle = lift['goal'](-.004)
    assert handle.accepted
    result = lift['wait'](handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0
    assert -.006 < lift['status']['lift_joint']['position'] < -.001
    assert not any(can_id == 1 and data[-1] == 0xfe for can_id, data in lift['motor'].frames)


@pytest.mark.parametrize('lift', [{'upper': True, 'initial': 11.5}], indirect=True)
def test_lift_unwraps_both_directions_and_limit_stops_are_directional(lift):
    m, io = lift['motor'], lift['io']
    io.bits = 0
    down = lift['goal'](-.004)
    assert down.accepted
    assert lift['wait'](down.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert min(m.raw_history) < -11
    assert -.01 < lift['status']['lift_joint']['position'] < -.001
    # Lower switch must stop downward velocity immediately, yet permit moving up.
    io.bits = 2
    lift['until'](lambda: lift['status']['lift_status']['lower'] == 1 and m.speeds[-1][1] == 0)
    marker = len(m.speeds)
    up = lift['goal'](0.0)
    assert up.accepted
    assert lift['wait'](up.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert any(speed < -.1 for _, speed in m.speeds[marker:])
    # Action goal tolerance is wider than the hardware hold tolerance; allow the
    # continuing position hold to finish crossing the encoder wrap boundary.
    lift['until'](lambda: m.raw_history[-1] > 0, timeout=5)
    assert m.raw_history[-1] > 0  # Crossed -12.5/+12.5 in the reverse direction.
    assert abs(lift['status']['lift_joint']['position']) < .002
    # Upper switch blocks upward motion even with a nonzero prior slew state.
    io.bits = 1
    lift['until'](lambda: lift['status']['lift_status']['upper'] == 1 and m.speeds[-1][1] == 0)
    assert lift['controllers']()['lift_controller'] == 'active'


@pytest.mark.parametrize('lift', [{'upper': True, 'poison_python': True}], indirect=True)
def test_lift_waiter_uses_ros_interpreter_when_conda_python_is_first(lift):
    assert lift['status']['lift_status']['homed'] == 1
    assert lift['controllers']()['lift_controller'] == 'active'
    assert 'wrong-python-ABI' not in lift['log'].read_text()


@pytest.mark.parametrize('lift', [{'upper': True}], indirect=True)
def test_lift_rejects_whole_invalid_goals_and_topics_without_losing_controller(lift):
    lift['until'](lift['action'].server_is_ready)
    marker = len(lift['motor'].speeds)
    for value in [.05, -.81, float('nan'), float('inf')]:
        handle = lift['goal'](value)
        assert not handle.accepted
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['lift_joint']
    first = JointTrajectoryPoint(positions=[-.001])
    first.time_from_start.nanosec = 200000000
    last = JointTrajectoryPoint(positions=[.05])
    last.time_from_start.sec = 1
    goal.trajectory.points = [first, last]
    assert not lift['wait'](lift['action'].send_goal_async(goal)).accepted
    # Invalid streaming commands must not bypass Action validation.
    publisher = lift['node'].create_publisher(JointTrajectory, f"/{lift['ns']}/lift_controller/joint_trajectory", 10)
    lift['until'](lambda: publisher.get_subscription_count() > 0)
    publisher.publish(goal.trajectory)
    deadline = time.monotonic() + .2
    lift['until'](lambda: time.monotonic() >= deadline, timeout=1)
    assert all(abs(v) < 1e-6 for _, v in lift['motor'].speeds[marker:])
    assert lift['controllers']() == {'lift_joint_state_broadcaster': 'active', 'lift_controller': 'active'}
    assert lift['status']['lift_status']['homed'] == 1
    handle = lift['goal'](-.004)
    assert handle.accepted
    assert lift['wait'](handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.parametrize('lift', [{'upper': True}], indirect=True)
def test_lift_invalid_replacement_does_not_cancel_current_valid_goal(lift):
    current = lift['goal'](-.02)
    assert current.accepted
    lift['until'](lambda: any(v > .1 for _, v in lift['motor'].speeds))
    assert not lift['goal'](.05).accepted
    assert lift['wait'](current.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert lift['controllers']()['lift_controller'] == 'active'


@pytest.mark.parametrize('lift', [{'upper': True}], indirect=True)
def test_lift_spline_overshoot_aborts_only_the_goal_and_recovers(lift):
    lift['io'].bits = 0  # Release the upper switch once descending away from home.
    lift['until'](lift['action'].server_is_ready)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['lift_joint']
    first = JointTrajectoryPoint(positions=[-.004], velocities=[.1])
    first.time_from_start.sec = 1
    last = JointTrajectoryPoint(positions=[-.004], velocities=[0.0])
    last.time_from_start.sec = 2
    goal.trajectory.points = [first, last]
    handle = lift['wait'](lift['action'].send_goal_async(goal))
    assert handle.accepted
    result = lift['wait'](handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert result.result.error_code == FollowJointTrajectory.Result.INVALID_GOAL
    assert 'interpolation' in result.result.error_string
    assert lift['controllers']() == {'lift_joint_state_broadcaster': 'active', 'lift_controller': 'active'}
    handle = lift['goal'](-.003)
    assert handle.accepted
    assert lift['wait'](handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED


@pytest.mark.parametrize('fault', ['stale', 'crc', 'both'])
@pytest.mark.parametrize('lift', [{'upper': True}], indirect=True)
def test_lift_io_faults_stop_owned_motor(lift, fault):
    m, io = lift['motor'], lift['io']
    io.bits = 0
    handle = lift['goal'](-.02)
    assert handle.accepted
    lift['until'](lambda: any(speed > .1 for _, speed in m.speeds))
    if fault == 'stale':
        io.paused = True
    elif fault == 'crc':
        io.corrupt = True
    else:
        io.bits = 3
    lift['until'](lambda: not m.enabled[1], timeout=3)
    result = lift['wait'](handle.get_result_async(), timeout=5)
    assert result.status == GoalStatus.STATUS_ABORTED
    assert lift['controllers']()['lift_controller'] != 'active'


@pytest.mark.parametrize('lift', [
    {'startup_failure': True, 'corrupt': True},
    {'startup_failure': True, 'home_timeout': .4},
], indirect=True)
def test_lift_missing_io_or_homing_timeout_never_leaves_motor_running(lift):
    lift['until'](lambda: lift['process'].poll() is not None, timeout=15, allow_exit=True)
    assert not lift['motor'].enabled[1]
    text = lift['log'].read_text()
    if lift['io'].corrupt:
        assert not any(can_id == 1 and data[-1] == 0xfc for can_id, data in lift['motor'].frames)
        assert 'no valid lift IO frame' in text
    else:
        assert 'homing timeout' in text
