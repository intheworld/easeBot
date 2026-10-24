#!/usr/bin/env python3
"""Read-only startup barrier for the four real actuator controller managers."""
import math
import time

import rclpy
from controller_manager_msgs.srv import ListControllers


def main():
    rclpy.init()
    node = rclpy.create_node('wait_robot_ready')
    readonly = node.declare_parameter('read_only', False).value
    timeout = float(node.declare_parameter('timeout_sec', 90.0).value)
    expected = {}
    for side in ('left', 'right'):
        expected[f'{side}_controller_manager'] = {f'{side}_joint_state_broadcaster': 'active',
            f'{side}_gripper_controller': 'inactive' if readonly else 'active'}
        if not readonly:
            expected[f'{side}_controller_manager'][f'{side}_arm_controller'] = 'active'
    for component in ('base', 'lift'):
        expected[f'{component}_controller_manager'] = {f'{component}_joint_state_broadcaster': 'active'}
        if not readonly:
            expected[f'{component}_controller_manager'][f'{component}_controller'] = 'active'
    clients = {name: node.create_client(ListControllers, f'{name}/list_controllers') for name in expected}
    pending, states, received = {}, {}, {}
    next_query = {name: 0.0 for name in expected}
    try:
        if not math.isfinite(timeout) or timeout <= 0:
            node.get_logger().error('startup timeout must be positive and finite')
            return 1
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            for name, client in clients.items():
                future = pending.get(name)
                if future is not None and future.done():
                    try:
                        states[name] = {c.name: c.state for c in future.result().controller}
                        received[name] = now
                    except Exception as error:
                        node.get_logger().warning(f'{name}: {error}')
                    del pending[name]
                if name not in pending and now >= next_query[name] and client.service_is_ready():
                    pending[name] = client.call_async(ListControllers.Request())
                    next_query[name] = now + .25
            if all(now - received.get(name, -math.inf) < 1.0 and
                   all(states.get(name, {}).get(c) == state for c, state in controllers.items())
                   for name, controllers in expected.items()):
                node.get_logger().info('Robot actuators ready: ' +
                    ('read-only feedback, grippers inactive' if readonly else
                     'both arms, both grippers, base and homed lift active'))
                return 0
            rclpy.spin_once(node, timeout_sec=.05)
        missing = {name: {c: states.get(name, {}).get(c, 'unavailable') for c, state in controllers.items()
                          if states.get(name, {}).get(c) != state} for name, controllers in expected.items()}
        node.get_logger().error(f'Robot actuator startup timed out: {missing}')
        return 1
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
