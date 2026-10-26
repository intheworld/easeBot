"""Policy runtime tests need neither ROS, MHS, an LLM nor physical hardware."""
import asyncio
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1] / 'src'

from hei_agent import (
    Agent, BaseNode, Edge, Executor, FunctionNode, Graph, GraphError,
    Node, NodeRegistry, NodeResult, RunStatus,
)
from hei_agent.mock import MockConnector


def demo():
    return Graph.load(PACKAGE / 'hei_agent/examples/named_pose.json')


def simple(kind='test', *, timeout=1, graph_timeout=2, edges=None, extra=()):
    return Graph('test', 'work',
                 (Node('work', kind, timeout_sec=timeout),
                  Node('done', 'end', {'status': 'succeeded'}), *extra),
                 tuple(edges if edges is not None else [Edge('work', 'done')]),
                 timeout_sec=graph_timeout)


def registered(function):
    return NodeRegistry().register('test', FunctionNode(function))


def test_demo_data_binding_trace_and_graph_round_trip(tmp_path):
    graph = Graph.from_dict(demo().to_dict())
    connector = MockConnector(delay_sec=0)
    result = asyncio.run(Agent(graph, connector).run({'side': 'left'}, trace_dir=tmp_path))
    assert result.status is RunStatus.SUCCEEDED
    assert result.outputs == {'last_pose': 'transport'}
    assert connector.poses == {'left': 'transport'}
    assert result.steps == 5
    assert len({call['call_id'] for call in connector.calls}) == 3
    events = [json.loads(line) for line in Path(result.trace_path).read_text().splitlines()]
    assert events == list(result.events)
    assert [e['sequence'] for e in events] == list(range(len(events)))
    assert events[0]['graph'] == graph.to_dict()
    assert events[-1]['status'] == 'succeeded'
    assert [e['node'] for e in events if e['event'] == 'node_started'] == [
        'observe', 'ready', 'approach', 'transport', 'done']


@pytest.mark.parametrize('ready,fail_at,expected_calls', [
    (False, None, 1), (True, 'pre_grasp', 2), (True, 'transport', 3),
])
def test_readiness_and_motion_failure_routes_stop_later_actions(ready, fail_at, expected_calls):
    connector = MockConnector(ready=ready, fail_at=fail_at, delay_sec=0)
    result = asyncio.run(Agent(demo(), connector).run({'side': 'right'}))
    assert result.status is RunStatus.FAILED
    assert len(connector.calls) == expected_calls
    assert result.events[-2]['node'] == 'failed'


@pytest.mark.parametrize('mutation', [
    lambda d: d.update(schema='workflow/v3'),
    lambda d: d.update(typo=True),
    lambda d: d.update(entry='missing'),
    lambda d: d.update(max_steps=True),
    lambda d: d.update(timeout_sec=float('nan')),
    lambda d: d['nodes'].append(d['nodes'][0]),
    lambda d: d['nodes'][0].update(timeout_sec=0),
    lambda d: d['nodes'][0].update(typo=True),
    lambda d: d['edges'].append(d['edges'][0]),
    lambda d: d['edges'][0].update(target='missing'),
    lambda d: d['edges'].append({'source': 'done', 'target': 'observe'}),
    lambda d: d['nodes'][1].update(inputs={'condition': {'$ref': 'nodes.absent.ready'}}),
    lambda d: d['nodes'][1].update(inputs={'condition': {'$ref': 'inputs.x', 'extra': 1}}),
    lambda d: d['nodes'].append({'id': 'unreachable', 'kind': 'end', 'inputs': {'status': 'failed'}}),
    lambda d: d['nodes'][-1].update(inputs={'status': 'unknown'}),
])
def test_invalid_graphs_rejected(mutation):
    data = demo().to_dict()
    mutation(data)
    with pytest.raises(GraphError):
        Graph.from_dict(data)


def test_duplicate_json_keys_rejected(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{"schema":"hei_agent.graph/v1","schema":"hei_agent.graph/v1"}')
    with pytest.raises(GraphError, match='Duplicate JSON key'):
        Graph.load(path)


def test_cycle_without_end_path_is_rejected():
    graph = Graph('closed_loop', 'loop', (Node('loop', 'test'),), (Edge('loop', 'loop'),))
    with pytest.raises(GraphError, match='possible path'):
        graph.validate()


def test_unknown_implementation_fails_before_any_skill_is_called():
    graph = demo()
    nodes = tuple(replace(n, kind='unregistered') if n.id == 'transport' else n for n in graph.nodes)
    connector = MockConnector()
    with pytest.raises(GraphError, match='Unregistered'):
        asyncio.run(Agent(replace(graph, nodes=nodes), connector).run({'side': 'right'}))
    assert connector.calls == []


def test_missing_input_does_not_dispatch_motion():
    connector = MockConnector(delay_sec=0)
    result = asyncio.run(Agent(demo(), connector).run())
    assert result.status is RunStatus.FAILED
    assert [c['skill'] for c in connector.calls] == ['get_robot_state']
    assert any('Unavailable data reference' in event.get('message', '') for event in result.events)


def test_unexecuted_branch_output_is_not_none_or_invented():
    graph = Graph('missing_output', 'choose', (
        Node('choose', 'branch', {'condition': False}),
        Node('unused', 'test'),
        Node('done', 'end', {'status': 'succeeded', 'outputs': {'x': {'$ref': 'nodes.unused.x'}}}),
    ), (Edge('choose', 'unused', 'true'), Edge('choose', 'done', 'false'), Edge('unused', 'done')))

    async def unused(context, inputs):
        pytest.fail('skipped branch executed')

    result = asyncio.run(Agent(graph, registry=registered(unused)).run())
    assert result.status is RunStatus.FAILED
    assert 'Unavailable data reference' in result.message


def test_exception_uses_explicit_error_edge():
    async def broken(context, inputs):
        raise RuntimeError('perception unavailable')

    graph = simple(edges=[Edge('work', 'done', 'error')])
    result = asyncio.run(Agent(graph, registry=registered(broken)).run())
    assert result.status is RunStatus.SUCCEEDED
    assert any('perception unavailable' in event.get('message', '') for event in result.events)


def test_unhandled_outcome_fails_instead_of_implicitly_succeeding():
    async def uncertain(context, inputs):
        return NodeResult('unknown')

    result = asyncio.run(Agent(simple(), registry=registered(uncertain)).run())
    assert result.status is RunStatus.FAILED
    assert "Unhandled outcome 'unknown'" in result.message


@pytest.mark.parametrize('invalid', [None, {'ok': True}, NodeResult(outputs={'x': float('nan')})])
def test_invalid_node_results_are_execution_errors(invalid):
    async def wrong(context, inputs):
        return invalid

    result = asyncio.run(Agent(simple(), registry=registered(wrong)).run())
    assert result.status is RunStatus.FAILED
    assert "Unhandled outcome 'error'" in result.message


def test_bounded_loop_and_per_visit_call_ids():
    seen = []

    async def again(context, inputs):
        seen.append(context.call_id)
        return NodeResult('again')

    graph = replace(simple(edges=[Edge('work', 'work', 'again'), Edge('work', 'done')]), max_steps=3)
    result = asyncio.run(Agent(graph, registry=registered(again)).run())
    assert result.status is RunStatus.FAILED
    assert result.steps == 3 and 'max_steps' in result.message
    assert len(set(seen)) == 3


def test_loop_can_recover_without_automatic_retry():
    async def sometimes(context, inputs):
        return NodeResult('retry' if context.visit == 1 else 'success')

    graph = simple(edges=[Edge('work', 'work', 'retry'), Edge('work', 'done')])
    result = asyncio.run(Agent(graph, registry=registered(sometimes)).run())
    assert result.status is RunStatus.SUCCEEDED and result.steps == 3


def test_timeout_cancels_external_call_before_following_timeout_edge():
    async def exercise():
        call_finished, cancel_entered, acknowledge = asyncio.Event(), asyncio.Event(), asyncio.Event()
        order = []

        class Connector:
            async def call(self, skill, arguments, *, call_id):
                try:
                    await asyncio.Event().wait()
                finally:
                    call_finished.set()

            async def cancel(self, call_id):
                cancel_entered.set()
                await acknowledge.wait()
                order.append('confirmed')

        async def next_node(context, inputs):
            assert call_finished.is_set()
            order.append('next')
            return NodeResult()

        graph = Graph('timeout', 'slow', (
            Node('slow', 'skill', {'skill': 'move', 'arguments': {}}, timeout_sec=0.02),
            Node('next', 'test'), Node('done', 'end', {'status': 'succeeded'}),
        ), (Edge('slow', 'next', 'timeout'), Edge('next', 'done')))
        agent = Agent(graph, Connector(), registered(next_node))
        running = asyncio.create_task(agent.run())
        await asyncio.wait_for(cancel_entered.wait(), 1)
        assert order == [] and not running.done()
        acknowledge.set()
        result = await running
        assert result.status is RunStatus.SUCCEEDED
        assert order == ['confirmed', 'next']

    asyncio.run(exercise())


def test_graph_deadline_is_terminal_even_when_node_has_timeout_route():
    async def slow(context, inputs):
        await asyncio.sleep(10)

    graph = simple(timeout=1, graph_timeout=0.02, edges=[Edge('work', 'done', 'timeout')])
    result = asyncio.run(Agent(graph, registry=registered(slow)).run())
    assert result.status is RunStatus.TIMED_OUT and result.steps == 1


def test_agent_cancel_and_busy_guard_then_reuse():
    async def exercise():
        started, finished = asyncio.Event(), asyncio.Event()

        async def slow(context, inputs):
            started.set()
            try:
                await asyncio.sleep(10)
            finally:
                finished.set()

        agent = Agent(simple(), registry=registered(slow))
        assert not agent.cancel()
        task = asyncio.create_task(agent.run())
        await started.wait()
        with pytest.raises(RuntimeError, match='busy'):
            await agent.run()
        assert agent.cancel()
        result = await task
        assert result.status is RunStatus.CANCELLED and result.cleanup_complete
        assert finished.is_set()
        assert not agent.cancel()
        agent.graph = Graph('end_only', 'done', (Node('done', 'end', {'status': 'succeeded'}),), ())
        assert (await agent.run()).status is RunStatus.SUCCEEDED

    asyncio.run(exercise())


def test_caller_task_cancellation_runs_cleanup_and_preserves_cancelled_error():
    async def exercise():
        started = asyncio.Event()
        cleanup = []

        class Slow(BaseNode):
            async def execute(self, context, inputs):
                started.set()
                await asyncio.sleep(10)

            async def cancel(self, context):
                cleanup.append(context.call_id)

        agent = Agent(simple(), registry=NodeRegistry().register('test', Slow()))
        task = asyncio.create_task(agent.run())
        await started.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert len(cleanup) == 1

    asyncio.run(exercise())


def test_caller_cancel_during_timeout_cleanup_does_not_send_duplicate_cancel():
    async def exercise():
        entered, release = asyncio.Event(), asyncio.Event()
        cancellations = []

        class Slow(BaseNode):
            async def execute(self, context, inputs):
                await asyncio.sleep(10)

            async def cancel(self, context):
                cancellations.append(context.call_id)
                entered.set()
                await release.wait()

        agent = Agent(simple(timeout=0.01), registry=NodeRegistry().register('test', Slow()))
        task = asyncio.create_task(agent.run())
        await asyncio.wait_for(entered.wait(), 1)
        task.cancel()
        await asyncio.sleep(0)
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert len(cancellations) == 1

    asyncio.run(exercise())


def test_graph_deadline_expiring_during_trace_does_not_dispatch_skill(monkeypatch):
    import time
    from hei_agent.trace import Trace

    original = Trace.record

    def slow_trace(self, event, **data):
        original(self, event, **data)
        if event == 'node_inputs':
            time.sleep(0.03)  # Model delayed disk I/O before the node task starts.

    monkeypatch.setattr(Trace, 'record', slow_trace)
    connector = MockConnector(delay_sec=0)
    result = asyncio.run(Agent(replace(demo(), timeout_sec=0.01), connector).run({'side': 'right'}))
    assert result.status is RunStatus.TIMED_OUT
    assert connector.calls == []


@pytest.mark.parametrize('hang', [False, True])
def test_unconfirmed_cancel_blocks_recovery_and_future_runs(hang):
    async def exercise():
        called = []

        class Connector:
            async def call(self, skill, arguments, *, call_id):
                called.append(skill)
                await asyncio.sleep(10)

            async def cancel(self, call_id):
                if hang:
                    await asyncio.sleep(10)
                raise RuntimeError('remote termination unknown')

        graph = replace(simple('skill', timeout=0.01, edges=[Edge('work', 'done', 'timeout')]),
                        nodes=(Node('work', 'skill', {'skill': 'move', 'arguments': {}}, 0.01),
                               Node('done', 'end', {'status': 'succeeded'})))
        agent = Agent(graph, Connector(), cancel_timeout_sec=0.02)
        result = await agent.run()
        assert result.status is RunStatus.FAILED and not result.cleanup_complete
        assert result.steps == 1 and called == ['move']
        with pytest.raises(RuntimeError, match='unconfirmed'):
            await agent.run()

    asyncio.run(exercise())


def test_slow_cancellation_handler_is_bounded_and_execution_is_quarantined():
    async def exercise():
        release = asyncio.Event()

        async def stubborn(context, inputs):
            try:
                await asyncio.sleep(10)
            except asyncio.CancelledError:
                await release.wait()
            return NodeResult()

        agent = Agent(simple(timeout=0.01), registry=registered(stubborn), cancel_timeout_sec=0.02)
        result = await asyncio.wait_for(agent.run(), 0.5)
        assert result.status is RunStatus.FAILED and not result.cleanup_complete
        release.set()
        await asyncio.sleep(0)
        with pytest.raises(RuntimeError, match='unconfirmed'):
            await agent.run()

    asyncio.run(exercise())


def test_sync_function_nodes_and_duplicate_builtin_registration_rejected():
    with pytest.raises(TypeError, match='async'):
        FunctionNode(lambda context, inputs: NodeResult())
    with pytest.raises(ValueError, match='already registered'):
        NodeRegistry().register('end', BaseNodeSubclass())


class BaseNodeSubclass(BaseNode):
    async def execute(self, context, inputs):
        return NodeResult()


def test_demo_cli_runs_without_install_or_ros_and_returns_failure_exit_code(tmp_path):
    command = [sys.executable, '-m', 'hei_agent', '--trace-dir', str(tmp_path)]
    # Isolate from the caller's ROS/Python overlay: only the source package is added.
    import os
    env = dict(os.environ, PYTHONPATH=str(PACKAGE))
    completed = subprocess.run(command, env=env, capture_output=True, text=True, timeout=5)
    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout)['status'] == 'succeeded'
    completed = subprocess.run(command + ['--not-ready'], env=env, capture_output=True, text=True, timeout=5)
    assert completed.returncode == 1, completed.stderr
    assert json.loads(completed.stdout)['status'] == 'failed'
