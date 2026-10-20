"""Send a small right-arm trajectory through the standard ROS action interface."""
import argparse
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description='Mock-only ROS 2 right arm action demo; does not enable controllers')
    parser.add_argument('--namespace', default='hei')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('hei_joint_demo', namespace=args.namespace)
    client = ActionClient(node, FollowJointTrajectory, 'right_arm_controller/follow_joint_trajectory')
    code = 1
    try:
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('action server unavailable; launch the mock stack first')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'a_right_joint{i}' for i in range(1, 7)]
        point = JointTrajectoryPoint()
        point.positions = [0.1, -0.5, -0.5, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('goal rejected or no response; enable mock controllers explicitly')
        handle = future.result()
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=8)
        if not result.done():
            handle.cancel_goal_async()
            raise RuntimeError('trajectory result timed out; cancellation requested')
        code = 0 if result.result().result.error_code == 0 else 1
        print(result.result().result)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if code:
        raise SystemExit(code)
