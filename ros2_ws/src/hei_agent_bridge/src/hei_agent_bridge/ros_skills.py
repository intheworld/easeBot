"""ROS action adapter. Creating this backend never enables a controller."""
import asyncio
from collections import deque
from statistics import median
import math
import threading
import time
from copy import deepcopy


async def ros_future(future):
    loop = asyncio.get_running_loop()
    result = loop.create_future()
    # A cancelled caller must not cancel an in-flight ROS send-goal operation.
    result.add_done_callback(lambda done: None if done.cancelled() else done.exception())

    def ready(completed):
        def deliver():
            if result.done():
                return
            try:
                result.set_result(completed.result())
            except Exception as exc:
                result.set_exception(exc)
        loop.call_soon_threadsafe(deliver)
    future.add_done_callback(ready)
    return await asyncio.shield(result)


class RosSkillBackend:
    def __init__(self, node, config):
        from control_msgs.action import FollowJointTrajectory, GripperCommand
        from control_msgs.msg import DynamicJointState
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.node, self.config = node, deepcopy(config)
        self.samples, self.stamps, self.lock = {}, {}, threading.Lock()
        self.operations = {}
        self.wheel_history = {}
        self.arm = {side: ActionClient(node, FollowJointTrajectory,
                    f'{side}_arm_controller/follow_joint_trajectory') for side in config['named_poses']
                    if config['named_poses'][side]}
        if ('right_grasp' in config or 'arm_motion' in config) and 'right' not in self.arm:
            self.arm['right'] = ActionClient(node, FollowJointTrajectory, 'right_arm_controller/follow_joint_trajectory')
        self.grippers = {side: ActionClient(node, GripperCommand,
                        f'{side}_gripper_controller/gripper_cmd') for side in ('left', 'right')}
        self.grasp_parameters = None
        if 'right_grasp' in config:
            from rcl_interfaces.srv import GetParameters
            self.grasp_parameters = node.create_client(GetParameters, 'right_gripper_controller/get_parameters')
        self.subscriptions = []
        for topic in ('joint_states', 'left_joint_states', 'right_joint_states'):
            self.subscriptions.append(node.create_subscription(JointState, topic,
                lambda msg, key=topic: self._joints(msg, key), qos_profile_sensor_data))
        for side in ('left', 'right', 'base', 'lift'):
            self.subscriptions.append(node.create_subscription(DynamicJointState, f'{side}_dynamic_joint_states',
                lambda msg, key=side: self._dynamic(msg, key), qos_profile_sensor_data))

    def _update(self, key, stamp, values):
        with self.lock:
            if self.stamps.get(key) == stamp:
                return
            self.stamps[key] = stamp
            for name, position, velocity in values:
                if math.isfinite(position) and math.isfinite(velocity):
                    now = time.monotonic()
                    self.samples[name] = (position, velocity, now)
                    if name.endswith('_wheel_joint'):
                        history = self.wheel_history.setdefault(name, deque(maxlen=256))
                        history.append((now, position, velocity))
                        while history and now-history[0][0] > .75:
                            history.popleft()

    def _joints(self, msg, key):
        if len(msg.name) != len(msg.position) or len(msg.name) != len(msg.velocity):
            return
        self._update(key, (msg.header.stamp.sec, msg.header.stamp.nanosec), zip(msg.name, msg.position, msg.velocity))

    def _dynamic(self, msg, key):
        values = []
        for name, interfaces in zip(msg.joint_names, msg.interface_values):
            state = dict(zip(interfaces.interface_names, interfaces.values))
            if 'position' in state and 'velocity' in state:
                values.append((name, state['position'], state['velocity']))
        self._update('dynamic_' + key, (msg.header.stamp.sec, msg.header.stamp.nanosec), values)

    @staticmethod
    def joints(side):
        prefix = 'a_right' if side == 'right' else 'b_left'
        return [f'{prefix}_joint{i}' for i in range(1, 7)]

    def _feedback(self, names, *, at_rest=False, target=None):
        with self.lock:
            samples = [self.samples.get(name) for name in names]
        if any(item is None or time.monotonic() - item[2] > 1 for item in samples):
            return False
        if at_rest and any(abs(item[1]) > 0.05 for item in samples):
            return False
        return target is None or all(abs(item[0] - goal) <= 0.03 for item, goal in zip(samples, target))

    def ready(self):
        return (all(client.server_is_ready() and self._feedback(self.joints(side)) for side, client in self.arm.items())
                and all(client.server_is_ready() and self._feedback([f'{side}_gripper_motor'])
                        for side, client in self.grippers.items()))

    async def _execute(self, invocation, client, goal, joints):
        if not client.server_is_ready() or not self._feedback(joints):
            raise RuntimeError('Action server or fresh position/velocity feedback unavailable')
        operation = {'send': client.send_goal_async(goal), 'joints': joints, 'handle': None, 'result': None}
        self.operations[invocation] = operation
        handle = await ros_future(operation['send'])
        operation['handle'] = handle
        if not handle.accepted:
            self.operations.pop(invocation, None)
            raise RuntimeError('ROS controller rejected goal')
        operation['result'] = handle.get_result_async()
        result = await ros_future(operation['result'])
        if result.status != 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            raise RuntimeError(f'ROS action ended with status {result.status}')
        return result.result

    async def _verify(self, joints, target=None):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if self._feedback(joints, at_rest=True, target=target):
                return
            await asyncio.sleep(0.02)
        raise RuntimeError('Fresh at-rest/target feedback could not be confirmed')

    async def move(self, invocation, side, pose):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        names = self.joints(side)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in pose['positions_rad']]
        sec, nsec = divmod(round(pose['duration_sec'] * 1e9), 1_000_000_000)
        point.time_from_start.sec, point.time_from_start.nanosec = sec, nsec
        goal.trajectory.points = [point]
        result = await self._execute(invocation, self.arm[side], goal, names)
        if result.error_code != 0:
            raise RuntimeError(f'Trajectory failed: {result.error_code}')
        await self._verify(names, pose['positions_rad'])
        self.operations.pop(invocation, None)

    async def gripper(self, invocation, side, gap):
        from control_msgs.action import GripperCommand
        names = [f'{side}_gripper_motor']
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = gap, 0.0
        result = await self._execute(invocation, self.grippers[side], goal, names)
        if not result.reached_goal or not math.isfinite(result.position) or abs(result.position - gap) > 0.002:
            raise RuntimeError('Gripper did not reach requested nominal gap')
        await self._verify(names)
        self.operations.pop(invocation, None)
        return result.position

    async def grasp_check(self, profile):
        from rcl_interfaces.srv import GetParameters
        client = self.grasp_parameters
        if client is None or not client.service_is_ready():
            raise RuntimeError('Right gripper parameter service unavailable')
        names = ['allow_stalling', 'gap_at_zero_m', 'gap_per_motor_rad_m',
                 'default_current_ratio', 'max_current_ratio']
        response = await asyncio.wait_for(ros_future(client.call_async(GetParameters.Request(names=names))), 2)
        values = response.values
        if (len(values) != 5 or values[0].type != 1 or not values[0].bool_value or
                any(v.type != 3 or not math.isfinite(v.double_value) for v in values[1:])):
            raise RuntimeError('Right gripper must support allow_stalling and finite position/current parameters')
        zero, slope, current, maximum = [v.double_value for v in values[1:]]
        if abs(zero) > 1e-9 or abs(slope-profile['gap_per_motor_rad_m']) > 1e-9 or not 0 < current <= maximum <= .05:
            raise RuntimeError('Grasp/ROS gripper gap mapping or current limit mismatch')

    def _wheel_rest_locked(self, names, now):
        """0.5s encoder evidence plus median velocity; caller owns samples lock.

        DM velocity feedback has quantization bias and occasional isolated spikes.
        A position span of .005 rad is .25 mm at the configured .05 m wheel radius.
        Actual accumulated displacement is also checked against the plan's start.
        """
        for name in names:
            window = [v for v in self.wheel_history.get(name, ()) if now-v[0] <= .5]
            if len(window) < 3 or window[-1][0]-window[0][0] < .25 or now-window[-1][0] > .2:
                raise RuntimeError(f'Fresh stationary wheel history unavailable: {name}')
            span = max(v[1] for v in window)-min(v[1] for v in window)
            speed = median(v[2] for v in window)
            if span > .005 or abs(speed) > .05:
                raise RuntimeError(f'Grasp scene is moving: wheels; {name}: '
                                   f'position_span_rad={span:.6f}, median_velocity_rad_s={speed:.6f}')

    def arm_pose_state(self):
        with self.lock:
            names = self.joints('right')+['lift_joint']
            now = time.monotonic()
            samples = [self.samples.get(name) for name in names]
            if any(v is None or now-v[2] > 1 for v in samples):
                raise RuntimeError('Fresh right-arm/lift pose feedback unavailable')
            return {'right':[v[0] for v in samples[:6]], 'lift':samples[6][0]}

    def grasp_state(self, *, at_rest=True):
        return self.arm_scene_state(at_rest=at_rest, require_gripper=True)

    def arm_scene_state(self, *, at_rest=True, require_gripper=False):
        wheels = ['front_right_wheel_joint', 'rear_right_wheel_joint',
                  'rear_left_wheel_joint', 'front_left_wheel_joint']
        groups = {'right': self.joints('right'), 'left': self.joints('left'),
                  'lift': ['lift_joint'], 'right_gripper': ['right_gripper_motor'],
                  'left_gripper': ['left_gripper_motor'], 'wheels': wheels}
        if require_gripper and ('right' not in self.arm or not self.arm['right'].server_is_ready() or
                                not self.grippers['right'].server_is_ready()):
            raise RuntimeError('Right arm/gripper action servers unavailable')
        result = {}
        with self.lock:
            now = time.monotonic()
            for key, names in groups.items():
                samples = [self.samples.get(name) for name in names]
                if any(v is None or now-v[2] > 1 for v in samples):
                    raise RuntimeError(f'Fresh grasp scene feedback unavailable: {key}')
                if key == 'wheels':
                    self._wheel_rest_locked(names, now)
                elif (at_rest or key not in ('right', 'right_gripper')) and any(abs(v[1]) > .05 for v in samples):
                    raise RuntimeError(f'Grasp scene is moving: {key}')
                values = [v[0] for v in samples]
                result[key] = values[0] if len(values) == 1 else values
        return result

    async def grasp_trajectory(self, invocation, start, waypoints, speed):
        await self.arm_trajectory(invocation,start,waypoints,speed)

    async def arm_trajectory(self, invocation, start, waypoints, speed):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal = FollowJointTrajectory.Goal()
        names = self.joints('right')
        goal.trajectory.joint_names = names
        from .trajectory import JointTrajectory
        trajectory = await asyncio.to_thread(JointTrajectory, start, waypoints, speed)
        for q, velocity, acceleration, elapsed in zip(
                trajectory.positions, trajectory.velocities, trajectory.accelerations, trajectory.times):
            point = JointTrajectoryPoint(positions=q.tolist(), velocities=velocity.tolist(),
                                         accelerations=acceleration.tolist())
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(round(float(elapsed)*1e9), 1_000_000_000)
            goal.trajectory.points.append(point)
        result = await self._execute(invocation, self.arm['right'], goal, names)
        if result.error_code != 0:
            raise RuntimeError(f'Arm trajectory failed: {result.error_code}')
        await self._verify(names, waypoints[-1])
        self.operations.pop(invocation, None)

    async def grasp_gripper(self, invocation, gap, *, close):
        from control_msgs.action import GripperCommand
        names = ['right_gripper_motor']
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = float(gap), 0.0
        result = await self._execute(invocation, self.grippers['right'], goal, names)
        if not result.reached_goal and not (close and result.stalled):
            raise RuntimeError('Grasp gripper target not reached or closing stall not confirmed')
        if not math.isfinite(result.position) or result.position < 0 or (not close and abs(result.position-gap) > .002):
            raise RuntimeError('Invalid grasp gripper gap feedback')
        await self._verify(names)
        self.operations.pop(invocation, None)
        return bool(result.stalled)

    async def cancel(self, invocation):
        operation = self.operations.get(invocation)
        if operation is None:
            return
        handle = operation['handle'] or await ros_future(operation['send'])
        if handle.accepted:
            await ros_future(handle.cancel_goal_async())
            result = operation['result'] or handle.get_result_async()
            terminal = await ros_future(result)
            if terminal.status not in (4, 5, 6):
                raise RuntimeError('ROS goal termination not confirmed')
            await self._verify(operation['joints'])
        self.operations.pop(invocation, None)

    def close(self):
        if self.grasp_parameters is not None:
            self.node.destroy_client(self.grasp_parameters)
        for client in list(self.arm.values()) + list(self.grippers.values()):
            client.destroy()
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
