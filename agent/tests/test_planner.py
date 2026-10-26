"""Planning validation and repair without a model service or robot."""
import asyncio
import json
from pathlib import Path

import pytest

from hei_agent import Agent, Graph, GraphError, GraphPlanner, LLMConfig, LLMError
from hei_agent.catalog import MOCK_CATALOG
from hei_agent.llm import ModelReply
from hei_agent.mock import MockConnector
from hei_agent.planner import validate_plan


EXAMPLE = Path(__file__).resolve().parents[1] / 'src/hei_agent/examples/named_pose.json'


def graph_data():
    return json.loads(EXAMPLE.read_text())


class ScriptedModel:
    def __init__(self, *responses):
        self.responses = list(responses)
        self.messages = []

    async def complete(self, messages):
        self.messages.append(list(messages))
        response = self.responses.pop(0)
        if isinstance(response, Exception):
            raise response
        return ModelReply(response, 'scripted-model', {'total_tokens': 12})


def test_generation_saves_inspectable_graph_before_separate_execution(tmp_path):
    async def exercise():
        model = ScriptedModel(EXAMPLE.read_text())
        planner = GraphPlanner(model, MOCK_CATALOG)
        plan = await planner.generate('右臂先到预抓取位，再到搬运位', {'side': 'right'}, directory=tmp_path)
        assert plan.attempts == 1
        assert Graph.load(plan.graph_path) == plan.graph
        assert json.loads((plan.directory / 'result.json').read_text())['status'] == 'generated'
        request = json.loads((plan.directory / 'request.json').read_text())
        assert request['skills'] == MOCK_CATALOG.describe()
        assert 'JSON' in request['messages'][0]['content']
        assert not list(tmp_path.rglob('*.jsonl'))
        result = await Agent(plan.graph, MockConnector(delay_sec=0)).run({'side': 'right'})
        assert result.status.value == 'succeeded'

    asyncio.run(exercise())


def test_invalid_graph_can_be_repaired_without_executing_a_skill(tmp_path):
    invalid = graph_data()
    invalid['nodes'][2]['inputs']['arguments']['pose'] = 'invented'
    model = ScriptedModel(json.dumps(invalid), '```json\n' + EXAMPLE.read_text() + '\n```')
    plan = asyncio.run(GraphPlanner(model, MOCK_CATALOG).generate('移动右臂', {'side': 'right'}, directory=tmp_path))
    assert plan.attempts == 2
    first = json.loads((plan.directory / 'attempt-1.json').read_text())
    assert not first['valid'] and 'expected string' in first['error']
    assert model.messages[1][-2]['role'] == 'assistant'
    assert 'Validation failed' in model.messages[1][-1]['content']
    assert not list(tmp_path.rglob('*.jsonl'))


def test_invalid_generation_has_bounded_repairs_and_no_valid_graph(tmp_path):
    model = ScriptedModel('not JSON', 'still not JSON')
    with pytest.raises(LLMError, match='after 2 attempt'):
        asyncio.run(GraphPlanner(model, MOCK_CATALOG).generate('任务', {}, directory=tmp_path))
    assert len(model.messages) == 2
    assert not list(tmp_path.rglob('graph.json'))
    results = list(tmp_path.rglob('result.json'))
    assert json.loads(results[0].read_text())['status'] == 'failed'


def test_api_failure_is_not_retried_as_graph_repair(tmp_path):
    model = ScriptedModel(LLMError('HTTP failure'), EXAMPLE.read_text())
    with pytest.raises(LLMError, match='HTTP failure'):
        asyncio.run(GraphPlanner(model, MOCK_CATALOG).generate('任务', {}, directory=tmp_path))
    assert len(model.messages) == 1


def test_unsupported_task_can_produce_an_explicit_failed_plan(tmp_path):
    response = json.dumps({'schema': 'hei_agent.graph/v1', 'name': 'unsupported', 'entry': 'failed',
                           'nodes': [{'id': 'failed', 'kind': 'end', 'inputs': {
                               'status': 'failed', 'message': 'Catalog has no grasping skill'}}], 'edges': []})
    plan = asyncio.run(GraphPlanner(ScriptedModel(response), MOCK_CATALOG).generate('抓住杯子', directory=tmp_path))
    result = asyncio.run(Agent(plan.graph).run())
    assert result.status.value == 'failed'
    assert 'no grasping' in result.message


@pytest.mark.parametrize('mutation', [
    lambda d: d['nodes'][0].update(kind='execute_python'),
    lambda d: d['nodes'][0]['inputs'].update(skill='execute_trajectory'),
    lambda d: d['nodes'][0]['inputs'].update(skill={'$ref': 'inputs.skill'}),
    lambda d: d['nodes'][2]['inputs']['arguments'].update(side='back'),
    lambda d: d['nodes'][2]['inputs']['arguments'].update(pose='pick'),
    lambda d: d['nodes'][2]['inputs']['arguments'].update(speed=100),
    lambda d: d['nodes'][1]['inputs'].update(condition='true'),
    lambda d: d['nodes'][1]['inputs'].update(condition={'$ref': 'nodes.observe.invented'}),
    lambda d: d['nodes'][2]['inputs']['arguments'].update(side={'$ref': 'inputs.missing'}),
    lambda d: d['nodes'][2]['inputs']['arguments'].update(pose={'$ref': 'nodes.transport.pose'}),
    lambda d: d['nodes'][2]['inputs']['arguments'].update(side={'$ref': 'nodes.observe.ready'}),
    lambda d: d['edges'][3].update(target='done'),  # Skips output producer on false path.
    lambda d: d['edges'][1].update(target='ready'),  # Error path has no observe.ready.
    lambda d: d.update(max_steps=1000),
    lambda d: d['nodes'][0].update(timeout_sec=100),
])
def test_planning_preflight_rejects_unknown_capabilities_types_and_invalid_dependencies(mutation):
    data = graph_data()
    mutation(data)
    with pytest.raises(GraphError):
        validate_plan(Graph.from_dict(data), {'side': 'right'}, MOCK_CATALOG)


def test_catalog_typed_output_can_flow_to_another_skill():
    data = graph_data()
    data['nodes'][3]['inputs']['arguments']['side'] = {'$ref': 'nodes.approach.side'}
    validate_plan(Graph.from_dict(data), {'side': 'left'}, MOCK_CATALOG)


def test_planner_cancellation_saves_cancelled_status(tmp_path):
    async def exercise():
        started = asyncio.Event()

        class WaitingModel:
            async def complete(self, messages):
                started.set()
                await asyncio.sleep(10)

        task = asyncio.create_task(GraphPlanner(WaitingModel(), MOCK_CATALOG).generate('任务', directory=tmp_path))
        await started.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert json.loads(next(tmp_path.rglob('result.json')).read_text())['status'] == 'cancelled'

    asyncio.run(exercise())


def test_config_reads_environment_and_does_not_repr_secret(monkeypatch):
    monkeypatch.setenv('LLM_MODEL', 'my-model')
    monkeypatch.setenv('LLM_BASE_URL', 'http://127.0.0.1:8000/v1')
    monkeypatch.setenv('LLM_API_KEY', 'fake-test-secret')
    monkeypatch.setenv('LLM_TIMEOUT_SEC', '12')
    config = LLMConfig.from_env(model='override')
    assert config.model == 'override' and config.timeout_sec == 12
    assert 'fake-test-secret' not in repr(config)


def test_config_rejects_invalid_transport_environment_option(monkeypatch):
    monkeypatch.setenv('LLM_MODEL', 'test')
    monkeypatch.setenv('LLM_TRUST_ENV', 'maybe')
    with pytest.raises(LLMError, match='LLM_TRUST_ENV'):
        LLMConfig.from_env()


@pytest.mark.parametrize('changes', [
    {'model': ''}, {'timeout_sec': 0}, {'timeout_sec': float('nan')},
    {'base_url': 'file:///tmp/model'}, {'base_url': 'https://user:key@example.com/v1'},
    {'base_url': 'https://example.com/v1?key=secret'}, {'response_format': 'tools'},
])
def test_invalid_config_rejected(changes):
    values = {'model': 'test', 'base_url': 'http://127.0.0.1:8000/v1', **changes}
    with pytest.raises(ValueError):
        LLMConfig(**values)


def test_real_openai_endpoint_requires_credentials():
    with pytest.raises(LLMError, match='API_KEY'):
        LLMConfig(model='user-selected-model')


def test_blank_project_key_can_fall_back_to_openai_key(monkeypatch):
    monkeypatch.setenv('LLM_MODEL', 'user-selected-model')
    monkeypatch.setenv('LLM_API_KEY', '')
    monkeypatch.setenv('OPENAI_API_KEY', 'fake-fallback-key')
    assert LLMConfig.from_env().api_key == 'fake-fallback-key'


def test_catalog_declared_long_grasp_timeouts_and_examples():
    # Remote scalar contract: Agent tests must not import ROS/vision dependencies.
    from hei_agent.catalog import SkillCatalog, SkillSpec, ValueSpec
    from hei_agent.graph import Graph
    from hei_agent.planner import validate_plan
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    text, boolean = ValueSpec('string'), ValueSpec('boolean')
    catalog = SkillCatalog([
        SkillSpec('get_camera_observation', 'camera', {'camera':text}, {'observation_id':text}, outcomes=('success','failure')),
        SkillSpec('perceive_grasp_target','perceive',{'observation_id':text,'target':text},
                  {'object_id':text,'obb_ref':text,'target':text,'frame_id':text,'geometry_scope':text,
                   **{k:ValueSpec('number') for k in ('x_m','y_m','z_m','size_x_m','size_y_m','size_z_m')}},
                  timeout_sec=90,outcomes=('success','failure')),
        SkillSpec('plan_right_grasp_from_object','plan',{'object_id':text},
                  {'plan_id':text,'plan_ref':text,'validated':boolean,'validation_status':text},
                  timeout_sec=180,outcomes=('success','failure')),
        SkillSpec('execute_right_grasp', 'execute', {'plan_id':text},
                  {'grasp_verified':boolean, 'gripper_stalled':boolean}, motion=True, timeout_sec=300, outcomes=('success','failure')),
    ])
    for name in ('right_object_obb.json','right_obb_grasp_plan.json','right_obb_grasp_execute.json'):
        graph = Graph.load(root / 'agent/src/hei_agent/examples' / name)
        validate_plan(graph, {}, catalog)
