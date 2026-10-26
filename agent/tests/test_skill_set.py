"""Cross-project contract and robot skill runtime tests; no ROS/SDK needed."""
import asyncio
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/hei_agent_bridge/src'))

from hei_agent import Agent, Graph, GraphError, GraphPlanner
from hei_agent.catalog import SkillCatalog
from hei_agent.llm import ModelReply
from hei_agent_bridge.skills import MockSkillBackend, SkillSet, load_config


def config():
    return load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json')


def test_robot_manifest_round_trip_and_configuration_sensitive_revision():
    first = SkillSet(config(), MockSkillBackend())
    manifest = first.manifest()
    catalog = SkillCatalog.from_manifest(manifest)
    assert catalog.revision == first.revision
    assert {skill['name'] for skill in catalog.describe()} == {
        'get_robot_state', 'move_to_named_pose', 'set_gripper_gap'}
    updated = config()
    updated['named_poses']['right']['pre_grasp']['positions_rad'][0] = 0.25
    assert first.revision != SkillSet(updated, MockSkillBackend()).revision
    first.config['named_poses'].clear()
    assert first.config['named_poses']  # Configuration snapshots cannot mutate implementations.
    manifest['skills'][0]['description'] = 'changed remotely'
    with pytest.raises(GraphError, match='digest mismatch'):
        SkillCatalog.from_manifest(manifest)


def test_ros_config_has_no_invented_named_pose_skill():
    settings = load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.ros.example.json')
    skillset = SkillSet(settings, MockSkillBackend())
    assert {entry['name'] for entry in skillset.manifest()['skills']} == {'get_robot_state', 'set_gripper_gap'}
    with pytest.raises(RuntimeError, match='immutable'):
        skillset.register('extra', '', {}, {}, lambda: None)


def test_all_three_skills_execute_and_validate_outputs():
    async def exercise():
        backend = MockSkillBackend(delay=0)
        skills = SkillSet(config(), backend)
        for name, arguments in [
            ('get_robot_state', {}),
            ('move_to_named_pose', {'side': 'right', 'pose': 'transport'}),
            ('set_gripper_gap', {'side': 'left', 'gap_m': 0.025}),
        ]:
            result = await skills.invoke(name, arguments, invocation=name, revision=skills.revision)
            assert result['outcome'] == 'success' and result['cleanup_complete']
            SkillCatalog.from_manifest(skills.manifest()).get(name).validate_outputs(result['outputs'])
        assert backend.gaps['left'] == 0.025
        assert backend.positions['right'][0] == 0.1

    asyncio.run(exercise())


@pytest.mark.parametrize('name,arguments,revision', [
    ('unknown', {}, None), ('get_robot_state', {'extra': 1}, None),
    ('move_to_named_pose', {'side': 'right', 'pose': 'invented'}, None),
    ('set_gripper_gap', {'side': 'left', 'gap_m': -0.01}, None),
    ('set_gripper_gap', {'side': 'left', 'gap_m': True}, None),
    ('set_gripper_gap', {'side': 'left', 'gap_m': 0.5}, None),
    ('get_robot_state', {}, '0' * 64),
])
def test_invalid_call_never_reaches_backend(name, arguments, revision):
    skills = SkillSet(config(), MockSkillBackend())
    with pytest.raises(ValueError):
        asyncio.run(skills.invoke(name, arguments, invocation='test', revision=revision or skills.revision))
    assert skills.backend.calls == []


def test_duplicate_invocation_and_cancel_before_dispatch_never_replay():
    async def exercise():
        skills = SkillSet(config(), MockSkillBackend(delay=0))
        await skills.invoke('get_robot_state', {}, invocation='same', revision=skills.revision)
        with pytest.raises(ValueError, match='already used'):
            await skills.invoke('get_robot_state', {}, invocation='same', revision=skills.revision)
        assert (await skills.cancel('late'))['cleanup_complete']
        with pytest.raises(ValueError, match='already used'):
            await skills.invoke('move_to_named_pose', {'side': 'right', 'pose': 'transport'},
                                invocation='late', revision=skills.revision)
        assert not skills.backend.calls

    asyncio.run(exercise())


def test_motion_busy_cancel_and_reuse():
    async def exercise():
        backend = MockSkillBackend(delay=10)
        skills = SkillSet(config(), backend)
        task = asyncio.create_task(skills.invoke('move_to_named_pose', {'side': 'right', 'pose': 'transport'},
                                  invocation='first', revision=skills.revision))
        while not backend.calls:
            await asyncio.sleep(0)
        with pytest.raises(RuntimeError, match='busy'):
            await skills.invoke('set_gripper_gap', {'side': 'left', 'gap_m': 0.01},
                                invocation='second', revision=skills.revision)
        result = await skills.cancel('first')
        assert result['status'] == 'cancelled' and result['cleanup_complete']
        await task
        assert not skills.backend.positions and skills.motion_owner is None
        backend.delay = 0
        assert (await skills.invoke('set_gripper_gap', {'side': 'left', 'gap_m': 0.01},
                                    invocation='third', revision=skills.revision))['status'] == 'succeeded'

    asyncio.run(exercise())


def test_termination_failure_blocks_all_subsequent_motion():
    class Backend(MockSkillBackend):
        async def cancel(self, invocation):
            raise RuntimeError('stop unconfirmed')

    async def exercise():
        skills = SkillSet(config(), Backend(delay=10))
        task = asyncio.create_task(skills.invoke('set_gripper_gap', {'side': 'right', 'gap_m': 0.01},
                                  invocation='first', revision=skills.revision))
        while not skills.backend.calls:
            await asyncio.sleep(0)
        result = await skills.cancel('first')
        await task
        assert not result['cleanup_complete'] and skills.blocked
        with pytest.raises(RuntimeError, match='unconfirmed'):
            await skills.invoke('set_gripper_gap', {'side': 'left', 'gap_m': 0.01},
                                invocation='next', revision=skills.revision)

    asyncio.run(exercise())


def test_cancel_racing_with_local_worker_start_does_not_leave_running_record():
    async def exercise():
        skills = SkillSet(config(), MockSkillBackend(delay=10))
        task = asyncio.create_task(skills.invoke('set_gripper_gap', {'side': 'right', 'gap_m': 0.01},
                                  invocation='first', revision=skills.revision))
        await asyncio.sleep(0)  # Invocation registered; _perform has not started yet.
        result = await skills.cancel('first')
        await asyncio.gather(task, return_exceptions=True)
        assert result['status'] == 'not_started' and result['cleanup_complete']
        assert not skills.backend.calls and skills.motion_owner is None

    asyncio.run(exercise())


def test_planner_binds_revision_and_executor_rejects_changed_catalog(tmp_path):
    async def exercise():
        skills = SkillSet(config(), MockSkillBackend())
        catalog = SkillCatalog.from_manifest(skills.manifest())
        graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/named_pose.json')

        class Model:
            async def complete(self, messages):
                return ModelReply(json.dumps(graph.to_dict()), 'test')

        plan = await GraphPlanner(Model(), catalog).generate('task', {'side': 'right'}, directory=tmp_path)
        assert plan.graph.catalog_revision == skills.revision
        connector = MockSkillBackend()
        connector.catalog = SkillCatalog([], revision='0' * 64)
        with pytest.raises(GraphError, match='revision differs'):
            await Agent(plan.graph, connector).run({'side': 'right'})

    asyncio.run(exercise())
