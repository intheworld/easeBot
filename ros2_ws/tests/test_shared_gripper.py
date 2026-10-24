"""One real controller_manager + one DamiaoSystem + seven PTY motors; no hardware."""
import os
import math
from pathlib import Path
import signal
import struct
import subprocess
import time
import uuid

import pytest
import yaml
rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces, SwitchController
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectoryPoint
from test_damiao_stack import UsbCanAdapter, ROOT


class SharedAdapter(UsbCanAdapter):
    def __init__(self, motors):
        self.mode = 1
        self.arm_mode = 2
        self.follow_gripper = True
        self.drop_gripper = False
        self.overlapped = False
        super().__init__(motors)
        self.positions[7] = -.1

    def handle(self, can_id, data):
        if self.drop_gripper and (can_id in (7, 0x307) or
                (can_id == 0x7ff and int.from_bytes(data[:2], 'little') == 7)):
            with self.lock:
                self.frames.append((can_id, data))
            return
        if can_id == 0x7ff and int.from_bytes(data[:2], 'little') == 7:
            if data[2] == 0x55:
                with self.lock:
                    self.frames.append((can_id, data))
                assert data[3:] == b'\x0a\x04\x00\x00\x00'
                self.mode = 4
                self.reply(self.motors[7], data)
                return
            if data[2:4] == b'\x33\x0a':
                with self.lock:
                    self.frames.append((can_id, data))
                self.reply(self.motors[7], data[:4] + struct.pack('<I', self.mode))
                return
        if can_id == 0x307:
            with self.lock:
                self.frames.append((can_id, data))
            target, speed, current = struct.unpack('<fHH', data)
            assert -4.5 <= target <= 0 and 1 <= speed <= 1000 and 1 <= current <= 500
            assert self.mode == 4
            if self.enabled[7]:
                self.overlapped |= all(self.enabled[i] for i in range(1, 7))
                if self.follow_gripper:
                    self.positions[7] = target
            self.status(7)
            return
        if can_id == 0x7ff and data[2:4] == b'\x33\x0a':
            motor_id = int.from_bytes(data[:2], 'little')
            with self.lock:
                self.frames.append((can_id, data))
            self.reply(self.motors[motor_id], data[:4] + struct.pack('<I', self.arm_mode))
            return
        super().handle(can_id, data)


@pytest.fixture
def stack(tmp_path, monkeypatch, request):
    settings = getattr(request, 'param', {})
    settings = settings if isinstance(settings, dict) else {'calibrated': bool(settings)}
    calibrated = settings.get('calibrated', False)
    readonly = settings.get('readonly', False)
    debug = settings.get('debug', False)
    config = yaml.safe_load((ROOT / 'hei_bringup/config/right_arm.yaml').read_text())
    adapter = SharedAdapter(config['motors'] + [config['gripper']])
    if debug:
        adapter.arm_mode = 1
    config['port'] = adapter.path
    config['feedback_timeout_ms'] = 500
    config_path = tmp_path / 'bus.yaml'
    config_path.write_text(yaml.safe_dump(config))
    controllers = yaml.safe_load((ROOT / 'hei_control/config/damiao_gripper.yaml').read_text())
    controllers['/**/right_gripper_controller']['ros__parameters']['calibrated'] = calibrated
    if calibrated:
        # Synthetic calibration belongs ONLY to this PTY test, never hardware config.
        controllers['/**/right_gripper_controller']['ros__parameters'].update(
            calibrated=True, gap_at_zero_m=.045, gap_per_motor_rad_m=.01,
            force_per_motor_torque_n_nm=100.0, motor_torque_per_current_ratio_nm=10.0)
    if settings.get('claimed_calibrated'):
        controllers['/**/right_gripper_controller']['ros__parameters']['calibrated'] = True
    controllers['/**/right_gripper_controller']['ros__parameters']['goal_timeout_sec'] = settings.get('goal_timeout_sec', 0.0)
    controller_path = tmp_path / 'gripper.yaml'
    controller_path.write_text(yaml.safe_dump(controllers))
    domain = 120 + os.getpid() % 30
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('shared_bus_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    ns = 'shared_' + uuid.uuid4().hex[:8]
    log_path = tmp_path / 'shared.log'
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'damiao_arm.launch.py', 'activate_arm:=false',
        f'namespace:={ns}', f'config_file:={config_path}', f'gripper_config_file:={controller_path}',
        f'read_only:={str(readonly).lower()}',
        *(['gripper_position_only:=false', 'activate_gripper:=false', 'configure_control_mode:=false'] if not debug else []),
        *(['arm_read_only:=true', 'activate_gripper:=true', 'gripper_position_only:=true'] if debug else [])], stdout=log, stderr=log, env=os.environ.copy(), start_new_session=True)

    def until(predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            assert adapter.error is None, repr(adapter.error)
            assert process.poll() is None, log_path.read_text()[-18000:]
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        raise AssertionError('shared bus timeout\n' + log_path.read_text()[-18000:])

    def wait(future, timeout=20):
        until(future.done, timeout)
        return future.result()

    def call(client, request):
        until(client.service_is_ready)
        return wait(client.call_async(request))

    listing = node.create_client(ListControllers, f'/{ns}/controller_manager/list_controllers')
    switching = node.create_client(SwitchController, f'/{ns}/controller_manager/switch_controller')
    interfaces = node.create_client(ListHardwareInterfaces, f'/{ns}/controller_manager/list_hardware_interfaces')
    standard = ActionClient(node, GripperCommand, f'/{ns}/right_gripper_controller/gripper_cmd')
    arm = ActionClient(node, FollowJointTrajectory, f'/{ns}/right_arm_controller/follow_joint_trajectory')

    def switch(activate=(), deactivate=()):
        req = SwitchController.Request()
        req.activate_controllers, req.deactivate_controllers = list(activate), list(deactivate)
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = 3
        return call(switching, req).ok

    def trajectory(delta):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'a_right_joint{i}' for i in range(1, 7)]
        point = JointTrajectoryPoint(positions=[delta, -.5-delta, -.5-delta, delta, delta, delta])
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        return wait(arm.send_goal_async(goal))

    try:
        until(lambda: all(c in {v.name for v in call(listing, ListControllers.Request()).controller}
                          for c in (('joint_state_broadcaster', 'right_gripper_controller') if readonly or debug else ('right_arm_controller', 'right_gripper_controller'))))
        if readonly:
            until(lambda: any(c.name == 'joint_state_broadcaster' and c.state == 'active'
                              for c in call(listing, ListControllers.Request()).controller))
        until(standard.server_is_ready)
        if debug and not readonly:
            until(lambda: any(c.name == 'right_gripper_controller' and c.state == 'active'
                              for c in call(listing, ListControllers.Request()).controller))
        configured = {c.name: c for c in call(listing, ListControllers.Request()).controller}
        assert configured['right_gripper_controller'].type == 'hei_hardware/GripperController'
        actions = {name: types for name, types in get_action_names_and_types(node)
                   if name.startswith(f'/{ns}/right_gripper_controller/')}
        assert actions == {f'/{ns}/right_gripper_controller/gripper_cmd':
                           ['control_msgs/action/GripperCommand']}
        info = call(interfaces, ListHardwareInterfaces.Request())
        assert len(info.command_interfaces) == (0 if readonly else 4 if debug else 10)  # 6 arm + 4 gripper
        assert len(info.state_interfaces) == 23
        assert not any(adapter.enabled.values())  # Configuration did not enable motors.
        assert adapter.mode == (1 if readonly else 4)
        yield adapter, standard, wait, until, switch, trajectory, call, listing
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
        for client in (standard, arm):
            client.destroy()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        adapter.close()
    assert adapter.error is None


def gripper_goal(position, effort=10.0):
    goal = GripperCommand.Goal()
    goal.command.position, goal.command.max_effort = position, effort
    return goal


def test_uncalibrated_standard_action_rejects_without_enabling_gripper(stack):
    adapter, standard, wait, until, switch, _, _, _ = stack
    assert switch(activate=['right_gripper_controller'])
    rejected = wait(standard.send_goal_async(gripper_goal(.02, 1.0)))
    assert not rejected.accepted
    assert not any(adapter.enabled.values())


@pytest.mark.parametrize('stack', [{'calibrated': True, 'goal_timeout_sec': .3}], indirect=True)
def test_arm_and_gripper_share_bus_hold_cancel_timeout_and_independent_deactivation(stack):
    adapter, standard, wait, until, switch, trajectory, _, _ = stack
    assert switch(activate=['right_arm_controller', 'right_gripper_controller'])
    until(lambda: all(adapter.enabled[i] for i in range(1, 7)))
    assert not adapter.enabled[7]  # Gripper waits for a goal even while controller active.
    for position, effort in ((.05, 10.0), (.02, -1.0), (.02, 51.0), (float('nan'), 1.0)):
        assert not wait(standard.send_goal_async(gripper_goal(position, effort))).accepted
    assert not adapter.enabled[7]
    arm_goal = trajectory(.02)
    assert arm_goal.accepted
    feedback = []
    handle = wait(standard.send_goal_async(gripper_goal(.041), feedback_callback=feedback.append))
    assert handle.accepted
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.reached_goal
    assert abs(result.result.position-.041) < .0001 and feedback
    assert adapter.overlapped and all(adapter.enabled.values())  # Hold after success.
    assert wait(arm_goal.get_result_async()).result.error_code == 0
    adapter.follow_gripper = False
    canceled = wait(standard.send_goal_async(gripper_goal(.038)))
    assert canceled.accepted
    assert not wait(standard.send_goal_async(gripper_goal(.039))).accepted  # Busy.
    assert wait(canceled.cancel_goal_async()).goals_canceling
    assert wait(canceled.get_result_async()).status == GoalStatus.STATUS_CANCELED
    until(lambda: not adapter.enabled[7])
    assert all(adapter.enabled[i] for i in range(1, 7))
    timed = wait(standard.send_goal_async(gripper_goal(.037)))
    assert timed.accepted
    assert wait(timed.get_result_async()).status == GoalStatus.STATUS_ABORTED
    assert all(adapter.enabled[i] for i in range(1, 7)) and not adapter.enabled[7]
    assert switch(deactivate=['right_gripper_controller'])
    assert wait(trajectory(.03).get_result_async()).result.error_code == 0
    assert switch(deactivate=['right_arm_controller'])
    until(lambda: not any(adapter.enabled.values()))


@pytest.mark.parametrize('stack', [True], indirect=True)
def test_standard_action_with_synthetic_calibration_and_shared_hardware_fault(stack):
    adapter, standard, wait, until, switch, trajectory, call, listing = stack
    assert switch(activate=['right_arm_controller', 'right_gripper_controller'])
    handle = wait(standard.send_goal_async(gripper_goal(.04)))
    assert handle.accepted
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.reached_goal and abs(result.result.position-.04) < .0001
    assert abs(adapter.positions[7]+.5) < .001
    assert all(adapter.enabled.values())
    # A shared-bus hardware failure must stop both controllers, not leave the arm running.
    adapter.follow_gripper = False
    active = wait(standard.send_goal_async(gripper_goal(.037)))
    assert active.accepted
    adapter.drop_gripper = True
    result = wait(active.get_result_async(), timeout=6)
    assert result.status == GoalStatus.STATUS_ABORTED
    until(lambda: not any(adapter.enabled[i] for i in range(1, 7)))
    states = {c.name: c.state for c in call(listing, ListControllers.Request()).controller}
    assert states['right_arm_controller'] != 'active'
    assert states['right_gripper_controller'] != 'active'


@pytest.mark.parametrize('stack', [{'readonly': True}, {'readonly': True, 'debug': True}], indirect=True)
def test_shared_seven_motor_readonly_never_writes_or_enables(stack):
    adapter, _, _, _, _, _, call, listing = stack
    assert all(can_id == 0x7ff and data[2] == 0xcc for can_id, data in adapter.frames)
    assert {int.from_bytes(data[:2], 'little') for _, data in adapter.frames} == set(range(1, 8))
    assert not any(adapter.enabled.values())
    states = {c.name: c.state for c in call(listing, ListControllers.Request()).controller}
    assert states == {'joint_state_broadcaster': 'active', 'right_gripper_controller': 'inactive'}


@pytest.mark.parametrize('stack', [{'debug': True, 'calibrated': True}], indirect=True)
def test_gripper_debug_autostarts_with_mit_arms_state_only(stack):
    adapter, standard, wait, until, switch, _, call, listing = stack
    arm_positions = {i: adapter.positions[i] for i in range(1, 7)}
    states = {c.name: c.state for c in call(listing, ListControllers.Request()).controller}
    assert states == {'joint_state_broadcaster': 'active', 'right_gripper_controller': 'active'}
    assert not any(adapter.enabled.values())
    handle = wait(standard.send_goal_async(gripper_goal(.041)))
    assert handle.accepted
    assert wait(handle.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
    assert adapter.enabled[7] and not any(adapter.enabled[i] for i in range(1, 7))
    assert arm_positions == {i: adapter.positions[i] for i in range(1, 7)}
    assert all(can_id == 0x7ff and data[2] == 0xcc
               for can_id, data in adapter.frames
               if can_id not in (7, 0x307) and not (can_id == 0x7ff and int.from_bytes(data[:2], 'little') == 7))
    assert switch(deactivate=['right_gripper_controller'])
    until(lambda: not adapter.enabled[7])


@pytest.mark.parametrize('stack', [{'debug': True}], indirect=True)
def test_gripper_debug_active_does_not_bypass_calibration(stack):
    adapter, standard, wait, _, _, _, call, listing = stack
    states = {c.name: c.state for c in call(listing, ListControllers.Request()).controller}
    assert states['right_gripper_controller'] == 'active'
    assert not wait(standard.send_goal_async(gripper_goal(.002, 1.0))).accepted
    assert not any(adapter.enabled.values())


@pytest.mark.parametrize('stack', [{'debug': True}, {'debug': True, 'claimed_calibrated': True}], indirect=True)
def test_position_only_accepts_zero_effort_and_reports_force_unknown(stack):
    adapter, standard, wait, _, _, _, _, _ = stack
    handle = wait(standard.send_goal_async(gripper_goal(.002, 0.0)))
    assert handle.accepted
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert abs(result.result.position-.002) < .0002
    assert math.isnan(result.result.effort)
    commands = [struct.unpack('<fHH', data) for can_id, data in adapter.frames if can_id == 0x307]
    assert commands and all(speed == 1000 and current == 500 for _, speed, current in commands)  # 10 rad/s, 5% current ratio
    assert not wait(standard.send_goal_async(gripper_goal(.002, 1.0))).accepted
    assert not result.result.stalled
    assert adapter.enabled[7] and not any(adapter.enabled[i] for i in range(1, 7))


@pytest.mark.parametrize('stack', [{'debug': True}], indirect=True)
def test_rated_torque_threshold_accepts_old_trip_and_stops_above_rating(stack, tmp_path):
    adapter, standard, wait, until, _, _, call, listing = stack
    adapter.follow_gripper = False
    adapter.torques[7] = .6  # Above the retired 0.5 Nm threshold, below rated 3 Nm.
    feedback = []
    handle = wait(standard.send_goal_async(gripper_goal(.04, 0.0), feedback_callback=feedback.append))
    assert handle.accepted
    until(lambda: adapter.enabled[7] and len(feedback) >= 3)
    assert any(c.name == 'right_gripper_controller' and c.state == 'active'
               for c in call(listing, ListControllers.Request()).controller)
    adapter.torques[7] = 3.1
    assert wait(handle.get_result_async(), timeout=6).status == GoalStatus.STATUS_ABORTED
    until(lambda: not any(adapter.enabled.values()))
    log = (tmp_path / 'shared.log').read_text()
    assert 'torque_exceeded=1' in log and 'torque_limit_nm=3.000000' in log
