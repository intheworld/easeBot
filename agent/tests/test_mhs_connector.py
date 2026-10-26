"""Transport-independent tests for remote skill discovery and invocation identity."""
import asyncio
from dataclasses import replace
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent import Agent, Graph
from hei_agent.mhs import MHSConnector
from hei_agent_bridge.skills import MockSkillBackend, SkillSet, load_config


class LoopbackHub:
    def __init__(self, skills):
        self.skills = skills

    async def invoke(self, device, tool, arguments, **kwargs):
        if tool == 'get_skill_catalog':
            data = self.skills.manifest()
        elif tool == 'cancel_skill':
            data = await self.skills.cancel(arguments['invocation_id'])
        else:
            arguments = dict(arguments)
            revision, invocation = arguments.pop('revision'), arguments.pop('invocation_id')
            data = await self.skills.invoke(tool, arguments, invocation=invocation, revision=revision)
        return {'status': 'done', 'data': data}


def skillset(backend=None):
    config = load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json')
    return SkillSet(config, backend or MockSkillBackend(delay=0))


def test_server_graph_executes_robot_owned_skills():
    async def exercise():
        skills = skillset()
        connector = MHSConnector(LoopbackHub(skills), 'robot')
        catalog = await connector.discover()
        graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/named_pose.json')
        graph = replace(graph, catalog_revision=catalog.revision)
        result = await Agent(graph, connector).run({'side': 'right'})
        assert result.status.value == 'succeeded'
        assert skills.backend.positions['right'][0] == 0.1

    asyncio.run(exercise())


def test_remote_cancel_and_late_dispatch_tombstone():
    async def exercise():
        skills = skillset(MockSkillBackend(delay=10))
        dispatch_gate = asyncio.Event()

        class Hub(LoopbackHub):
            async def invoke(self, device, tool, arguments, **kwargs):
                if tool == 'move_to_named_pose':
                    await dispatch_gate.wait()
                return await super().invoke(device, tool, arguments, **kwargs)

        connector = MHSConnector(Hub(skills), 'robot')
        await connector.discover()
        task = asyncio.create_task(connector.call('move_to_named_pose', {'side': 'right', 'pose': 'transport'}, call_id='call'))
        await asyncio.sleep(0)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await connector.cancel('call')
        dispatch_gate.set()
        assert not skills.backend.calls
        assert all(record['result']['cleanup_complete'] for record in skills.records.values())

    asyncio.run(exercise())


def test_changed_robot_configuration_is_rejected_before_motion():
    async def exercise():
        first = skillset()
        hub = LoopbackHub(first)
        connector = MHSConnector(hub, 'robot')
        await connector.discover()
        config = first.config
        config['gripper_max_gap_m'] = 0.08
        hub.skills = SkillSet(config, MockSkillBackend())
        with pytest.raises(ValueError, match='revision changed'):
            await connector.call('set_gripper_gap', {'side': 'right', 'gap_m': 0.02}, call_id='test')
        assert not hub.skills.backend.calls

    asyncio.run(exercise())


@pytest.mark.parametrize('backend', ['mhs', 'embedded'])
def test_cli_compiles_unbound_template_against_discovered_catalog(tmp_path, backend):
    from argparse import Namespace
    import json
    from hei_agent.cli import run_policy
    async def exercise():
        skills = skillset()
        connector = MHSConnector(LoopbackHub(skills), 'robot')
        await connector.discover()
        source = ROOT / 'agent/src/hei_agent/examples/named_pose.json'
        original = source.read_bytes()
        args = Namespace(list_skills=False, task=None, graph=str(source), side='right',
                         backend=backend, validate=False, trace_dir=str(tmp_path))
        output, code = await run_policy(args, connector)
        assert code == 0 and output['status'] == 'succeeded'
        assert skills.backend.positions['right'][0] == .1
        trace = [json.loads(line) for line in Path(output['trace']).read_text().splitlines()]
        assert trace[0]['graph']['catalog_revision'] == skills.revision
        assert source.read_bytes() == original  # Do not rewrite portable templates.
    asyncio.run(exercise())


@pytest.mark.parametrize('problem', ['stale_revision', 'unknown_skill', 'invalid_arguments'])
def test_cli_never_rebinds_or_executes_invalid_graph(tmp_path, problem):
    from argparse import Namespace
    import json
    from hei_agent.cli import run_policy
    async def exercise():
        skills = skillset()
        connector = MHSConnector(LoopbackHub(skills), 'robot')
        await connector.discover()
        graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/named_pose.json').to_dict()
        if problem == 'stale_revision':
            graph['catalog_revision'] = '0' * 64
        else:
            motion = next(n for n in graph['nodes'] if n['inputs'].get('skill') == 'move_to_named_pose')
            if problem == 'unknown_skill':
                motion['inputs']['skill'] = 'does_not_exist'
            else:
                motion['inputs']['arguments']['pose'] = 'does_not_exist'
        path = tmp_path / 'invalid.json'
        path.write_text(json.dumps(graph))
        args = Namespace(list_skills=False, task=None, graph=str(path), side='right',
                         backend='embedded', validate=False, trace_dir=str(tmp_path))
        with pytest.raises(ValueError):
            await run_policy(args, connector)
        assert not skills.backend.calls
        assert not skills.records  # Reject the entire graph before even observing.
    asyncio.run(exercise())


def test_cli_validate_compiles_template_without_dispatch(tmp_path):
    from argparse import Namespace
    from hei_agent.cli import run_policy
    async def exercise():
        skills = skillset()
        connector = MHSConnector(LoopbackHub(skills), 'robot')
        await connector.discover()
        args = Namespace(list_skills=False, task=None,
                         graph=str(ROOT / 'agent/src/hei_agent/examples/named_pose.json'), side='right',
                         backend='embedded', validate=True, trace_dir=str(tmp_path))
        output, code = await run_policy(args, connector)
        assert code == 0 and output['valid']
        assert not skills.records and not skills.backend.calls
    asyncio.run(exercise())
