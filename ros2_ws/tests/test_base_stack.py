"""Native omni controller and four VEL motors over a PTY; never real hardware."""
import os
import signal
import struct
import subprocess
import time
import uuid

import pytest
import yaml
rclpy = pytest.importorskip('rclpy')
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from geometry_msgs.msg import TwistStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from test_damiao_stack import ROOT, UsbCanAdapter


class BaseAdapter(UsbCanAdapter):
    def handle(self, can_id, data):
        if can_id - 0x200 in self.motors:
            with self.lock:
                self.frames.append((can_id, data))
            motor_id = can_id - 0x200
            (speed,) = struct.unpack('<f', data[:4])
            assert abs(speed) <= 9.00001
            assert self.modes[motor_id] == 3
            self.status(motor_id)
            return
        super().handle(can_id, data)


@pytest.mark.parametrize('readonly,activate', [(True, True), (False, True), (False, False)])
def test_base_vel_mode_twist_timeout_and_cleanup(tmp_path, monkeypatch, readonly, activate):
    config = yaml.safe_load((ROOT / 'hei_bringup/config/base.yaml').read_text())
    adapter = BaseAdapter(config['motors'])
    adapter.modes = {i: 1 for i in adapter.motors}
    config.update(port=adapter.path, feedback_timeout_ms=500)
    profile = tmp_path / 'base.yaml'
    profile.write_text(yaml.safe_dump(config))
    domain = 120 + os.getpid() % 30
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    namespace = 'base_test_' + uuid.uuid4().hex[:8]
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('base_acceptance', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log_path = tmp_path / 'base.log'
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'damiao_base.launch.py',
        f'namespace:={namespace}', f'config_file:={profile}', 'configure_control_mode:=true',
        f'read_only:={str(readonly).lower()}', f'activate_base:={str(activate).lower()}'],
        stdout=log, stderr=log, env=dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'roslog')), start_new_session=True)

    def until(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()[-18000:]
            assert adapter.error is None, repr(adapter.error)
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(log_path.read_text()[-18000:])

    def call(client, request):
        until(client.service_is_ready)
        future = client.call_async(request)
        until(future.done)
        return future.result()

    def speeds():
        return [(can_id - 0x200, struct.unpack('<f', data[:4])[0])
                for can_id, data in adapter.frames if can_id - 0x200 in adapter.motors]

    try:
        listing = node.create_client(ListControllers, f'/{namespace}/base_controller_manager/list_controllers')
        interfaces = node.create_client(ListHardwareInterfaces, f'/{namespace}/base_controller_manager/list_hardware_interfaces')
        expected = {'base_joint_state_broadcaster': 'active'}
        if not readonly:
            expected['base_controller'] = 'active' if activate else 'inactive'
        until(lambda: {c.name: c.state for c in call(listing, ListControllers.Request()).controller} == expected)
        info = call(interfaces, ListHardwareInterfaces.Request())
        assert len(info.state_interfaces) == 12
        wheel_commands = [i for i in info.command_interfaces if not i.name.startswith('base_controller/')]
        assert len(wheel_commands) == (0 if readonly else 4)
        assert all(i.name.endswith('/velocity') for i in info.command_interfaces)
        if readonly:
            assert set(adapter.modes.values()) == {1}
            assert all(can_id == 0x7ff and data[2] == 0xcc for can_id, data in adapter.frames)
        else:
            assert set(adapter.modes.values()) == {3}
            assert all(adapter.enabled.values()) == activate
            if activate:
                publisher = node.create_publisher(TwistStamped, f'/{namespace}/base_controller/cmd_vel', 10)
                until(lambda: publisher.get_subscription_count() > 0)
                message = TwistStamped()
                message.header.stamp = node.get_clock().now().to_msg()
                message.twist.linear.x = .01
                publisher.publish(message)
                until(lambda: len({i for i, v in speeds() if abs(v) > .01}) == 4)
                # No more commands: the native controller must send zero to all wheels.
                deadline = time.monotonic() + .7
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.02)
                recent = speeds()[-4:]
                assert len(recent) == 4 and {i for i, _ in recent} == {1, 2, 3, 4}
                assert all(abs(v) < 1e-6 for _, v in recent)
    except BaseException:
        print(log_path.read_text()[-18000:])
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
        adapter.close()
    assert not any(adapter.enabled.values())
    assert adapter.error is None
