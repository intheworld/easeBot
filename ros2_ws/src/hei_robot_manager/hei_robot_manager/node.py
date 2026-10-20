"""ROS-native controller lifecycle facade. No legacy IPC server is used."""
import json
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers, SwitchController
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

REQUIRED_JOINTS = {f'{side}_joint{i}' for side in ('a_right', 'b_left') for i in range(1, 7)} | {'lift_joint', 'a_right_end_joint_L_finger', 'b_left_end_joint_L_finger', 'front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint'}

MOTION_CONTROLLERS = ('right_arm_controller', 'left_arm_controller', 'right_gripper_controller',
                      'left_gripper_controller', 'lift_controller', 'base_controller')


def call_wait(client, request, timeout=4):
    if not client.wait_for_service(timeout_sec=0.5):
        raise RuntimeError('controller manager service unavailable')
    future = client.call_async(request)
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(timeout):
        # A ROS service timeout does not cancel the remote operation.
        raise TimeoutError('controller manager response timed out; outcome unknown')
    return future.result()


class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        self.declare_parameter('state_timeout_s', 1.0)
        self.declare_parameter('controller_query_timeout_s', 5.0)
        self.timeout = float(self.get_parameter('state_timeout_s').value)
        self.query_timeout = float(self.get_parameter('controller_query_timeout_s').value)
        if self.timeout <= 0 or self.query_timeout <= 0:
            raise ValueError('feedback/query timeouts must be positive')
        self.group = ReentrantCallbackGroup()
        self.operations = threading.Lock()
        self.cache_lock = threading.Lock()
        self.controllers = {}
        self.last_controllers = self.last_state = 0.0
        self.last_stamp = None
        self.polling = False
        self.fault = ''
        self.uncertain = False
        self.recovery_pending = None
        self.list_client = self.create_client(ListControllers, 'controller_manager/list_controllers', callback_group=self.group)
        self.switch_client = self.create_client(SwitchController, 'controller_manager/switch_controller', callback_group=self.group)
        self.create_subscription(JointState, 'joint_states', self.on_state, qos_profile_sensor_data, callback_group=self.group)
        self.diagnostics = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_service(SetBool, '~/set_enabled', self.set_enabled, callback_group=self.group)
        self.create_service(Trigger, '~/stop', self.stop, callback_group=self.group)
        self.create_timer(0.5, self.poll, callback_group=self.group)
        self.get_logger().info('MOCK hardware only; motion controllers start inactive. Web remains read-only.')

    def on_state(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_stamp or len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name) or not REQUIRED_JOINTS.issubset(msg.name):
            return
        import math
        if all(math.isfinite(x) for x in msg.position):
            with self.cache_lock:
                self.last_stamp = stamp
                self.last_state = time.monotonic()

    def active(self):
        with self.cache_lock:
            return [name for name in MOTION_CONTROLLERS if self.controllers.get(name) == 'active']

    def poll(self):
        if self.list_client.service_is_ready() and not self.polling:
            self.polling = True
            future = self.list_client.call_async(ListControllers.Request())
            def done(result):
                self.polling = False
                try:
                    response = result.result()
                    with self.cache_lock:
                        self.controllers = {c.name: c.state for c in response.controller}
                        self.last_controllers = time.monotonic()
                except Exception as exc:
                    self.get_logger().warning(str(exc))
            future.add_done_callback(done)
        now = time.monotonic()
        stale = now - self.last_state > self.timeout
        if stale and self.active():
            self.fault = 'joint state feedback stale; requesting controller deactivation'
            self.uncertain = True
        if self.uncertain and self.recovery_pending is None and self.switch_client.service_is_ready():
            req = SwitchController.Request()
            req.deactivate_controllers = list(MOTION_CONTROLLERS)
            req.strictness = SwitchController.Request.BEST_EFFORT
            req.timeout.sec = 2
            self.recovery_pending = self.switch_client.call_async(req)
        if self.recovery_pending is not None and self.recovery_pending.done():
            try:
                if not self.recovery_pending.result().ok:
                    self.get_logger().warning('stop request failed; state remains uncertain')
                elif not self.active():
                    self.uncertain = False
            except Exception as exc:
                self.get_logger().warning(str(exc))
            self.recovery_pending = None
        self.publish_diagnostics(now, stale)

    def change_enabled(self, enabled):
        if not self.operations.acquire(blocking=False):
            raise RuntimeError('another controller operation is pending')
        try:
            state = call_wait(self.list_client, ListControllers.Request())
            controllers = {c.name: c.state for c in state.controller}
            if enabled:
                if self.uncertain or self.fault:
                    raise RuntimeError('stop/reset via set_enabled false before re-enabling')
                if time.monotonic() - self.last_state > self.timeout:
                    raise RuntimeError('valid joint feedback required')
                if not all(controllers.get(n) in ('active', 'inactive') for n in MOTION_CONTROLLERS):
                    raise RuntimeError('motion controllers are not all configured')
            req = SwitchController.Request()
            req.strictness = SwitchController.Request.STRICT
            req.timeout.sec = 2
            req.activate_controllers = [n for n in MOTION_CONTROLLERS if enabled and controllers.get(n) != 'active']
            req.deactivate_controllers = [n for n in MOTION_CONTROLLERS if not enabled and controllers.get(n) == 'active']
            if req.activate_controllers or req.deactivate_controllers:
                try:
                    response = call_wait(self.switch_client, req)
                except TimeoutError:
                    self.uncertain = True
                    self.fault = 'controller switch timed out; deactivation required'
                    raise
                if not response.ok:
                    self.uncertain = True
                    self.fault = 'controller switch failed'
                    raise RuntimeError(response.message)
            with self.cache_lock:
                self.controllers = controllers
                for n in req.activate_controllers:
                    self.controllers[n] = 'active'
                for n in req.deactivate_controllers:
                    self.controllers[n] = 'inactive'
                self.last_controllers = time.monotonic()
            if not enabled:
                self.fault = ''
                self.uncertain = False
            return 'mock motion controllers activated' if enabled else 'motion controllers inactive; not a hardware emergency stop'
        finally:
            self.operations.release()

    def set_enabled(self, request, response):
        try:
            response.message = self.change_enabled(request.data)
            response.success = True
        except Exception as exc:
            response.message, response.success = str(exc), False
        return response

    def stop(self, request, response):
        try:
            response.message = self.change_enabled(False)
            response.success = True
        except Exception as exc:
            response.message, response.success = str(exc), False
        return response

    def publish_diagnostics(self, now, stale):
        active = self.active()
        with self.cache_lock:
            controllers = dict(self.controllers)
            manager_age = now - self.last_controllers
            state_age = now - self.last_state
        ready = all(controllers.get(n) in ('active', 'inactive') for n in MOTION_CONTROLLERS)
        if self.fault or self.uncertain:
            mode, level = 'FAULT', DiagnosticStatus.ERROR
        elif stale or manager_age > self.query_timeout or not ready:
            mode, level = 'NOT_READY', DiagnosticStatus.WARN
        else:
            mode = 'ENABLED' if len(active) == len(MOTION_CONTROLLERS) else 'DISABLED' if not active else 'PARTIAL'
            level = DiagnosticStatus.OK
        status = DiagnosticStatus(name='hei/robot_manager', hardware_id='HeiMockSystem', level=level,
                                  message=mode)
        values = {'backend': 'ros2_control_mock', 'hardware_connected': 'false',
                  'state_age_s': str(state_age), 'controller_manager_age_s': str(manager_age),
                  'controllers': json.dumps(controllers), 'fault': self.fault,
                  'emergency_stop': 'unknown', 'homed': 'unknown'}
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diagnostics.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
