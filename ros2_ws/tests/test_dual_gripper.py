"""Two independent PTYs and real controller managers, no physical devices."""
import os
import signal
import subprocess
import time
import uuid

import pytest
import yaml
rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from test_shared_gripper import SharedAdapter, gripper_goal
from test_damiao_stack import ROOT


@pytest.mark.parametrize('readonly,full,activate', [
    (False, False, True), (True, False, True),
    (False, True, True), (True, True, True), (False, True, False)])
def test_both_grippers_start_and_control_independent_buses(tmp_path, monkeypatch, readonly, full, activate):
    domain = 120 + os.getpid() % 30
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    adapters, profiles = {}, {}
    for side in ('left', 'right'):
        config = yaml.safe_load((ROOT / f'hei_bringup/config/{side}_arm.yaml').read_text())
        adapter = SharedAdapter(config['motors'] + [config['gripper']])
        adapter.arm_mode = 2 if full else 1
        adapter.positions[2] = .01201629638671875
        adapters[side] = adapter
        config.update(port=adapter.path, feedback_timeout_ms=500)
        profiles[side] = tmp_path / f'{side}.yaml'
        profiles[side].write_text(yaml.safe_dump(config))
    suffix = uuid.uuid4().hex[:8]
    namespace = f'dual_{suffix}'
    log_path = tmp_path / 'dual.log'
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('dual_gripper_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    log = log_path.open('w')
    process = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'damiao_arms.launch.py' if full else 'grippers.launch.py',
        *(['activate_arm:=false'] if not activate else []),
        f'read_only:={str(readonly).lower()}',
        f'namespace:={namespace}',
        *[f'{side}_config_file:={profile}' for side, profile in profiles.items()]],
        stdout=log, stderr=log, env=os.environ.copy(), start_new_session=True)

    def until(predicate, timeout=25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()[-16000:]
            assert all(a.error is None for a in adapters.values())
            executor.spin_once(timeout_sec=.02)
            if predicate():
                return
        raise AssertionError(log_path.read_text()[-16000:])

    def wait(future):
        until(future.done)
        return future.result()

    def call(client, request):
        until(client.service_is_ready)
        return wait(client.call_async(request))

    clients = {}
    try:
        for side in adapters:
            ns = namespace
            listing = node.create_client(ListControllers, f'/{ns}/{side}_controller_manager/list_controllers')
            interfaces = node.create_client(ListHardwareInterfaces, f'/{ns}/{side}_controller_manager/list_hardware_interfaces')
            client = ActionClient(node, GripperCommand, f'/{ns}/{side}_gripper_controller/gripper_cmd')
            clients[side] = client
            expected = {f'{side}_joint_state_broadcaster': 'active',
                        f'{side}_gripper_controller': 'inactive' if readonly else 'active'}
            if full and not readonly:
                expected[f'{side}_arm_controller'] = 'active' if activate else 'inactive'
            until(lambda: {c.name: c.state for c in call(listing, ListControllers.Request()).controller} == expected)
            until(client.server_is_ready)
            info = call(interfaces, ListHardwareInterfaces.Request())
            assert len(info.state_interfaces) == 23
            assert len(info.command_interfaces) == (0 if readonly else 10 if full else 4)
            if not full:
                assert all(i.name.startswith(f'{side}_gripper_motor/') for i in info.command_interfaces)
            assert all(adapters[side].enabled[i] == (full and activate and not readonly) for i in range(1, 7))
            assert not adapters[side].enabled[7]
        until(lambda: all(f'/{namespace}/{side}_{topic}' in dict(node.get_topic_names_and_types())
                          for side in adapters for topic in ('tf_static', 'joint_states', 'robot_description')))
        graph = node.get_node_names_and_namespaces()
        for side in adapters:
            for name in (f'{side}_controller_manager', f'{side}_joint_state_broadcaster',
                         f'{side}_gripper_controller', f'{side}_robot_state_publisher'):
                assert graph.count((name, f'/{namespace}')) == 1
        if readonly:
            for client in clients.values():
                assert not wait(client.send_goal_async(gripper_goal(.003, 0.0))).accepted
            assert all(can_id == 0x7ff and data[2] == 0xcc
                       for adapter in adapters.values() for can_id, data in adapter.frames)
        else:
            # Starting one side must not enable or change the other bus.
            left = wait(clients['left'].send_goal_async(gripper_goal(.003, 0.0)))
            assert left.accepted
            assert wait(left.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED
            assert adapters['left'].enabled[7] and not adapters['right'].enabled[7]
            # Keep both goals in flight to verify simultaneous control.
            left_future = clients['left'].send_goal_async(gripper_goal(.004, 0.0))
            right_future = clients['right'].send_goal_async(gripper_goal(.005, 0.0))
            handles = [wait(left_future), wait(right_future)]
            assert all(h.accepted for h in handles)
            assert all(wait(h.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED for h in handles)
            assert all(a.enabled[7] for a in adapters.values())
            assert all(a.enabled[i] == (full and activate) for a in adapters.values() for i in range(1, 7))
            if full and activate:
                goals = []
                for side, prefix in [('left', 'b_left'), ('right', 'a_right')]:
                    client = ActionClient(node, FollowJointTrajectory, f'/{namespace}/{side}_arm_controller/follow_joint_trajectory')
                    clients[f'{side}_arm'] = client
                    until(client.server_is_ready)
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory.joint_names = [f'{prefix}_joint{i}' for i in range(1, 7)]
                    goal.trajectory.points = [JointTrajectoryPoint(positions=[0.0, -.02, -.5, 0.0, 0.0, 0.0])]
                    goal.trajectory.points[0].time_from_start.sec = 1
                    goals.append(client.send_goal_async(goal))
                handles = [wait(future) for future in goals]
                assert all(h.accepted for h in handles)
                assert all(wait(h.get_result_async()).status == GoalStatus.STATUS_SUCCEEDED for h in handles)
            assert abs(adapters['left'].positions[7] - (-.18)) < .001
            assert abs(adapters['right'].positions[7] - (-.225)) < .001
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        log.close()
        for client in clients.values():
            client.destroy()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        for adapter in adapters.values():
            adapter.close()
    assert not any(a.enabled[i] for a in adapters.values() for i in a.enabled)
    assert all(a.error is None for a in adapters.values())


@pytest.mark.parametrize('collision', ['namespace', 'serial'])
def test_dual_launch_rejects_conflicts_before_starting_nodes(tmp_path, collision):
    import runpy
    from launch import LaunchContext
    setup = runpy.run_path(str(ROOT / 'hei_bringup/launch/grippers.launch.py'))['setup']
    context = LaunchContext()
    values = dict(namespace='test_dual',
                  read_only='false', position_only='true', activate_gripper='true',
                  gripper_config_file='')
    for side in ('left', 'right'):
        port = tmp_path / f'{side}_port'
        if collision == 'serial':
            port.symlink_to(tmp_path / 'same_device')
        profile = tmp_path / f'{side}.yaml'
        profile.write_text(yaml.safe_dump({'port': str(port)}))
        values[f'{side}_config_file'] = str(profile)
    if collision == 'namespace':
        values['namespace'] = 'invalid/namespace'
    context.launch_configurations.update(values)
    with pytest.raises(ValueError, match='namespace must be|different serial devices'):
        setup(context)
