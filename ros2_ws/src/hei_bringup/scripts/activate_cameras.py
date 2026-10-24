#!/usr/bin/env python3
"""Initialize camera lifecycle once using acknowledged services, not events."""
import math
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


def main():
    rclpy.init()
    node = rclpy.create_node('activate_cameras')
    timeout = float(node.declare_parameter('timeout_sec', 30.0).value)
    state = node.create_client(GetState, 'cameras/get_state')
    change = node.create_client(ChangeState, 'cameras/change_state')
    deadline = time.monotonic() + timeout

    def wait(predicate):
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=.05)
        raise RuntimeError('camera startup timed out')

    def call(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        response_deadline = min(deadline, time.monotonic() + 1.5)
        while rclpy.ok() and not future.done() and time.monotonic() < response_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not future.done():
            # Initial DDS response discovery may lag request discovery. Query
            # state again rather than assuming a timed-out transition failed.
            client.remove_pending_request(future)
            future.cancel()
            return None
        return future.result()

    try:
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout_sec must be positive and finite')
        while rclpy.ok() and time.monotonic() < deadline:
            response = call(state, GetState.Request())
            if response is None:
                continue
            current = response.current_state.id
            if current == State.PRIMARY_STATE_ACTIVE:
                node.get_logger().info('Cameras configured and active')
                return 0
            if current >= 10:  # A configure/activate request may still be finishing.
                rclpy.spin_once(node, timeout_sec=.05)
                continue
            if current not in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE):
                raise RuntimeError(f'camera lifecycle is not ready for startup: state={current}')
            request = ChangeState.Request()
            request.transition.id = (Transition.TRANSITION_CONFIGURE if
                current == State.PRIMARY_STATE_UNCONFIGURED else Transition.TRANSITION_ACTIVATE)
            response = call(change, request)
            if response is not None and not response.success:
                raise RuntimeError('camera lifecycle transition rejected; inspect camera diagnostics')
        raise RuntimeError('camera startup interrupted')
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 130
    except Exception as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
