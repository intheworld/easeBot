#!/usr/bin/env python3
"""Wait for hardware homing state. This process never commands a motor."""
import time
import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('wait_lift_ready')
    timeout = float(node.declare_parameter('timeout_sec', 35.0).value)
    ready = False
    failed = False

    def state(message):
        nonlocal ready, failed
        for name, interfaces in zip(message.joint_names, message.interface_values):
            if name == 'lift_status':
                values = dict(zip(interfaces.interface_names, interfaces.values))
                ready = values.get('homed') == 1.0 and values.get('io_online') == 1.0
                failed = values.get('phase') == 3.0

    node.create_subscription(DynamicJointState, 'lift_dynamic_joint_states', state, qos_profile_sensor_data)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and not ready and not failed and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if ready:
            node.get_logger().info('Lift homed; height controller may activate')
            return 0
        node.get_logger().error('Lift homing failed or timed out; height controller not activated')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
