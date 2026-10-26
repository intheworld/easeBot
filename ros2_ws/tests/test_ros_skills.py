"""Real ROS action clients against fake controllers in an isolated namespace."""
import asyncio
import os
from pathlib import Path
import sys
import threading
import time

import pytest

pytest.importorskip('rclpy')
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import DynamicJointState, InterfaceValue

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.ros_skills import RosSkillBackend
from hei_agent_bridge.skills import SkillSet, load_config


def test_ros_arm_gripper_feedback_and_cancel(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    context = Context()
    rclpy.init(args=[], context=context, domain_id=170 + os.getpid() % 20)
    namespace = f'/skill_test_{os.getpid()}'
    server = Node('fake_controllers', namespace=namespace, context=context)
    client = Node('skill_client', namespace=namespace, context=context)
    positions, delayed, dispatched = {}, {'value': False}, []
    received_trajectories = []
    group = ReentrantCallbackGroup()
    publishers, actions = [], []

    def arm(goal):
        dispatched.append('arm')
        received_trajectories.append(goal.request.trajectory)
        while delayed['value'] and not goal.is_cancel_requested:
            time.sleep(0.01)
        if goal.is_cancel_requested:
            goal.canceled()
        else:
            positions.update(zip(goal.request.trajectory.joint_names, goal.request.trajectory.points[-1].positions))
            goal.succeed()
        return FollowJointTrajectory.Result(error_code=0)

    def gripper(goal):
        dispatched.append('gripper')
        goal.succeed()
        result = GripperCommand.Result()
        result.position, result.reached_goal = goal.request.command.position, True
        return result

    for side in ('left', 'right'):
        for name in RosSkillBackend.joints(side) + [f'{side}_gripper_motor']:
            positions[name] = 0.0
        publisher = server.create_publisher(DynamicJointState, f'{side}_dynamic_joint_states', 10)
        publishers.append((side, publisher))
        actions.append(ActionServer(server, FollowJointTrajectory, f'{side}_arm_controller/follow_joint_trajectory',
            execute_callback=arm, cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=group))
        actions.append(ActionServer(server, GripperCommand, f'{side}_gripper_controller/gripper_cmd',
            execute_callback=gripper, cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=group))

    def feedback():
        for side, publisher in publishers:
            message = DynamicJointState()
            message.header.stamp = server.get_clock().now().to_msg()
            message.joint_names = RosSkillBackend.joints(side) + [f'{side}_gripper_motor']
            message.interface_values = [InterfaceValue(interface_names=['position', 'velocity'],
                values=[positions[name], 0.0]) for name in message.joint_names]
            publisher.publish(message)
    timer = server.create_timer(0.02, feedback, callback_group=group)
    config = load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json')
    config['backend'] = 'ros'  # Fake action servers only, on isolated domain and namespace.
    config['right_grasp'] = {}
    from rcl_interfaces.msg import ParameterValue
    from rcl_interfaces.srv import GetParameters
    def parameters(request, response):
        response.values = [ParameterValue(type=1, bool_value=True)] + [
            ParameterValue(type=3, double_value=v) for v in (0., -.1/4.5, .05, .05)]
        return response
    parameter_service = server.create_service(GetParameters, 'right_gripper_controller/get_parameters', parameters,
                                             callback_group=group)
    backend = RosSkillBackend(client, config)
    skills = SkillSet(config, backend)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()

    async def exercise():
        deadline = time.monotonic() + 8
        while not backend.ready():
            assert time.monotonic() < deadline, 'Fake ROS servers/feedback not ready'
            await asyncio.sleep(0.02)
        result = await skills.invoke('move_to_named_pose', {'side': 'right', 'pose': 'transport'},
                                     invocation='arm', revision=skills.revision)
        assert result['status'] == 'succeeded', result
        assert positions['a_right_joint1'] == 0.1
        result = await skills.invoke('set_gripper_gap', {'side': 'left', 'gap_m': 0.02},
                                     invocation='grip', revision=skills.revision)
        assert result['status'] == 'succeeded', result
        assert result['outputs']['position_m'] == 0.02
        await backend.grasp_check({'gap_per_motor_rad_m':-.1/4.5})
        with pytest.raises(RuntimeError, match='mapping'):
            await backend.grasp_check({'gap_per_motor_rad_m':-.01})
        # Grasp paths preserve waypoints with continuous interior derivatives.
        await backend.grasp_trajectory('path', [0.]*6, [[.02]*6, [.03]*6], .1)
        assert positions['a_right_joint1'] == .03
        from hei_agent_bridge.trajectory import JointTrajectory
        expected = JointTrajectory([0.]*6,[[.02]*6,[.03]*6],.1)
        for point,q,v,a,t in zip(received_trajectories[-1].points,expected.positions,
                                  expected.velocities,expected.accelerations,expected.times,strict=True):
            assert list(point.positions) == pytest.approx(q)
            assert list(point.velocities) == pytest.approx(v)
            assert list(point.accelerations) == pytest.approx(a)
            assert point.time_from_start.sec+point.time_from_start.nanosec*1e-9 == pytest.approx(t,abs=1e-9)
        assert received_trajectories[-1].points[1].velocities[0] > 0
        assert 'path' not in backend.operations
        assert await backend.grasp_gripper('close', 0., close=True) is False
        delayed['value'] = True
        running = asyncio.create_task(skills.invoke('move_to_named_pose', {'side': 'left', 'pose': 'transport'},
                                        invocation='cancel', revision=skills.revision))
        deadline = time.monotonic() + 3
        while 'cancel' not in backend.operations or backend.operations['cancel']['handle'] is None:
            assert time.monotonic() < deadline
            await asyncio.sleep(0.01)
        result = await skills.cancel('cancel')
        await running
        assert result['cleanup_complete'] and result['status'] == 'cancelled', result
        assert positions['b_left_joint1'] == 0.0
        timer.cancel()
        await asyncio.sleep(0.05)
        with backend.lock:
            backend.samples.clear()
        count = len(dispatched)
        result = await skills.invoke('set_gripper_gap', {'side': 'right', 'gap_m': 0.02},
                                     invocation='stale', revision=skills.revision)
        assert result['status'] == 'failed' and result['cleanup_complete']
        assert len(dispatched) == count

    try:
        asyncio.run(exercise())
    finally:
        delayed['value'] = False
        timer.cancel()
        executor.remove_node(client)
        executor.remove_node(server)
        time.sleep(0.05)
        executor.shutdown(timeout_sec=3)
        worker.join(timeout=3)
        backend.close()
        server.destroy_service(parameter_service)
        for action in actions:
            action.destroy()
        client.destroy_node()
        server.destroy_node()
        context.shutdown()
