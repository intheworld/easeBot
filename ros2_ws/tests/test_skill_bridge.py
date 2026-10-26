"""Real MHS SDK + loopback relay + server connector; all skills simulated."""
import asyncio
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
import os

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'agent/src'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/hei_agent_bridge/src'))
pytest.importorskip('agnes_mhs', reason='source scripts/env/agent-bridge.bash')
from agnes_mhs import errors
from websockets.asyncio.server import serve

from hei_agent import Agent, Graph
from hei_agent.mhs import HubClient, MHSConnector
from hei_agent.hub import EmbeddedHub
from hei_agent_bridge.cache import RobotCache
from hei_agent_bridge.skill_device import SkillDevice
from hei_agent_bridge.skills import load_config


@pytest.mark.parametrize('ending', ['refused', 4001, 4003])
def test_auto_discovery_preserves_terminal_sdk_errors(monkeypatch, ending):
    from hei_discovery import Endpoint

    async def exercise():
        registrations = []

        async def handler(ws):
            if ws.request.path != '/ws/mhs':
                async for _ in ws:
                    pass
                return
            request = json.loads(await ws.recv())
            registrations.append(request)
            if ending == 'refused':
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': request['id'], 'error': {
                    'code': -32602, 'message': 'refused'}}))
            else:
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': request['id'], 'result': {
                    'session': 'test', 'hub': {'name': 'test', 'version': '1'}, 'time': time.time()}}))
                await ws.recv()  # Initial state proves registration is complete.
                await ws.close(ending, 'terminal')

        async with serve(handler, '127.0.0.1', 0, subprotocols=['mhs.v1']) as server:
            url = f'ws://127.0.0.1:{server.sockets[0].getsockname()[1]}'

            class FixedDiscovery:
                def __init__(self, **kwargs):
                    pass
                async def __aenter__(self):
                    return self
                async def __aexit__(self, *args):
                    pass
                async def wait(self):
                    return Endpoint('test', (url,))

            monkeypatch.setattr('hei_discovery.HubDiscovery', FixedDiscovery)
            device = SkillDevice(RobotCache(), 'skill-test',
                load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json'), None)
            await asyncio.wait_for(device.device.run('auto'), 5)
            await device.stop()
            assert len(registrations) == 1

    asyncio.run(exercise())


def test_auto_discovery_robot_first_and_hub_restart(monkeypatch):
    from hei_discovery import HubAdvertisement
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')

    async def exercise():
        device = SkillDevice(RobotCache(), 'skill-test',
            load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json'), None)
        worker = asyncio.create_task(device.device.run('auto', hub_id='test-hub', interfaces='127.0.0.1'))
        running = None
        try:
            await asyncio.sleep(.1)  # Robot is already browsing before the Hub exists.
            async with EmbeddedHub('127.0.0.1', 0, device_id='skill-test') as first:
                async with HubAdvertisement(first.port, hub_id='test-hub', device_id='skill-test', interfaces='127.0.0.1'):
                    await first.wait_device('skill-test', 10)
                    connector = MHSConnector(first, 'skill-test')
                    await connector.discover()
                    device.skills.backend.delay = 10
                    running = asyncio.create_task(connector.call('set_gripper_gap',
                        {'side': 'left', 'gap_m': .01}, call_id='lost-connection'))
                    deadline = time.monotonic() + 3
                    while device.skills.motion_owner is None:
                        assert time.monotonic() < deadline
                        await asyncio.sleep(.01)
            await asyncio.gather(running, return_exceptions=True)
            deadline = time.monotonic() + 4
            while device.skills.motion_owner is not None:
                assert time.monotonic() < deadline
                await asyncio.sleep(.01)
            assert 'left' not in device.skills.backend.gaps
            ledger = device.skills
            async with EmbeddedHub('127.0.0.1', 0, device_id='skill-test') as second:
                async with HubAdvertisement(second.port, hub_id='test-hub', device_id='skill-test', interfaces='127.0.0.1'):
                    await second.wait_device('skill-test', 12)
                    assert not worker.done()
                    assert device.skills is ledger  # Locks/history survive transport replacement.
                    connector = MHSConnector(second, 'skill-test')
                    await connector.discover()
                    device.skills.backend.delay = 0
                    result = await connector.call('set_gripper_gap', {'side': 'right', 'gap_m': .02}, call_id='new')
                    assert result.outputs['position_m'] == .02
                    assert 'left' not in device.skills.backend.gaps  # Lost motion was never replayed.
        finally:
            await device.stop()
            worker.cancel()
            await asyncio.gather(worker, return_exceptions=True)

    asyncio.run(exercise())


def test_serve_cli_keeps_discovered_device_between_tasks(monkeypatch, tmp_path):
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')

    async def exercise():
        device = SkillDevice(RobotCache(), 'skill-test',
            load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json'), None)
        graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/named_pose.json')
        graph_path = tmp_path / 'graph.json'
        graph_path.write_text(json.dumps(graph.to_dict()))
        worker = asyncio.create_task(device.device.run('auto', hub_id='cli-test', interfaces='127.0.0.1'))
        env = dict(os.environ)
        env.pop('LD_PRELOAD', None)
        env['PYTHONPATH'] = str(ROOT / 'agent/src') + os.pathsep + env.get('PYTHONPATH', '')
        process = await asyncio.create_subprocess_exec(sys.executable, '-m', 'hei_agent',
            '--backend', 'embedded', '--serve', '--listen-host', '127.0.0.1', '--listen-port', '0',
            '--hub-id', 'cli-test', '--discovery-interface', '127.0.0.1', '--device-id', 'skill-test',
            '--wait-device', '10', '--trace-dir', str(tmp_path), env=env,
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        try:
            process.stdin.write(b'{"side":"left"}\n')
            await process.stdin.drain()
            assert json.loads(await asyncio.wait_for(process.stdout.readline(), 10))['status'] == 'error'
            process.stdin.write(json.dumps({'graph': str(graph_path), 'side': 'right'}).encode() + b'\n')
            await process.stdin.drain()
            output = json.loads(await asyncio.wait_for(process.stdout.readline(), 15))
            assert output['status'] == 'succeeded', output
            generation = device.device._generation
            process.stdin.write(b'{"list_skills":true}\n')
            await process.stdin.drain()
            output = json.loads(await asyncio.wait_for(process.stdout.readline(), 5))
            assert len(output['skills']) == 5
            assert process.returncode is None and not worker.done()
            assert device.device._generation == generation
            process.stdin.close()
            _, stderr = await asyncio.wait_for(process.communicate(), 5)
            assert process.returncode == 0, stderr.decode()
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()
            await device.stop()
            worker.cancel()
            await asyncio.gather(worker, return_exceptions=True)

    asyncio.run(exercise())


def test_embedded_hub_with_real_sdk(monkeypatch, tmp_path):
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')

    async def exercise():
        async with EmbeddedHub('127.0.0.1', 0, device_id='skill-test') as hub:
            device = SkillDevice(RobotCache(), 'skill-test',
                load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json'), None)
            worker = asyncio.create_task(device.device.run(f'ws://127.0.0.1:{hub.port}'))
            pump = asyncio.create_task(device.pump(5))
            try:
                await hub.wait_device('skill-test', 5)
                connector = MHSConnector(hub, 'skill-test')
                catalog = await connector.discover()
                graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/named_pose.json')
                result = await Agent(replace(graph, catalog_revision=catalog.revision), connector).run({'side': 'right'})
                assert result.status.value == 'succeeded'
                result = await connector.call('set_gripper_gap', {'side': 'left', 'gap_m': .025}, call_id='grip')
                assert result.outputs['position_m'] == .025
                connector.artifact_dir = tmp_path / 'observations'
                observation = await connector.call('get_camera_observation', {'camera': 'front'}, call_id='front')
                assert observation.outputs['simulated'] and observation.outputs['depth_available']
                assert Path(observation.outputs['rgb_ref']).read_bytes().startswith(b'\xff\xd8')
                assert Path(observation.outputs['depth_ref']).read_bytes().startswith(b'\x93NUMPY')
                assert json.loads(Path(observation.outputs['metadata_ref']).read_text())['depth_unit'] == 'm'
                point = await connector.call('sample_depth_point',
                    {'observation_id': observation.outputs['observation_id'], 'u': .5, 'v': .5}, call_id='depth')
                assert point.outputs['z_m'] == 1. and point.outputs['frame_id'].startswith('front_')
                depth_graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/camera_depth.json')
                result = await Agent(replace(depth_graph, catalog_revision=catalog.revision), connector).run({})
                assert result.status.value == 'succeeded' and result.outputs['z_m'] == 1.
                assert Path(result.outputs['rgb']).is_file()
                device.skills.backend.delay = 10
                running = asyncio.create_task(connector.call('set_gripper_gap',
                    {'side': 'left', 'gap_m': .01}, call_id='cancel'))
                deadline = time.monotonic() + 3
                while device.skills.motion_owner is None:
                    assert time.monotonic() < deadline
                    await asyncio.sleep(.01)
                running.cancel()
                await asyncio.gather(running, return_exceptions=True)
                await connector.cancel('cancel')
                assert device.skills.motion_owner is None
                assert device.skills.backend.gaps['left'] == .025
            finally:
                await device.stop()
                worker.cancel()
                pump.cancel()
                await asyncio.gather(worker, pump, return_exceptions=True)

    asyncio.run(exercise())


def test_embedded_cli_discovers_real_sdk(monkeypatch):
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')

    async def exercise():
        env = dict(os.environ)
        env.pop('LD_PRELOAD', None)
        env['PYTHONPATH'] = str(ROOT / 'agent/src') + os.pathsep + env.get('PYTHONPATH', '')
        process = await asyncio.create_subprocess_exec(sys.executable, '-m', 'hei_agent',
            '--backend', 'embedded', '--no-discovery', '--listen-host', '127.0.0.1', '--listen-port', '0',
            '--device-id', 'skill-test', '--list-skills', env=env,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        worker = None
        device = None
        try:
            line = await asyncio.wait_for(process.stderr.readline(), 5)
            assert line.startswith(b'MHS listening on 127.0.0.1:'), line.decode()
            address = line.decode().split()[3].rstrip(';')
            device = SkillDevice(RobotCache(), 'skill-test',
                load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json'), None)
            worker = asyncio.create_task(device.device.run('ws://' + address))
            stdout, stderr = await asyncio.wait_for(process.communicate(), 10)
            assert process.returncode == 0, stderr.decode()
            result = json.loads(stdout)
            assert result['backend'] == 'embedded' and result['execution_backend'] == 'mock'
            assert len(result['skills']) == 5
        finally:
            if process.returncode is None:
                process.kill()
                await process.wait()
            if device is not None:
                await device.stop()
            if worker is not None:
                worker.cancel()
                await asyncio.gather(worker, return_exceptions=True)

    asyncio.run(exercise())


def test_real_mhs_device_catalog_graph_and_gripper(monkeypatch, tmp_path):
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')

    async def exercise():
        registered = asyncio.Event()
        connections, pending, tasks = {}, {}, set()
        sequence = 0

        async def forward(ws, request):
            nonlocal sequence
            sequence += 1
            call, job = f'call-{sequence}', f'job-{sequence}'
            accepted = asyncio.get_running_loop().create_future()
            result = asyncio.get_running_loop().create_future()
            pending[call] = (accepted, result)
            try:
                await connections['device'].send(json.dumps({'jsonrpc': '2.0', 'id': call, 'method': 'mhs/call',
                    'params': {'name': request['params']['tool'], 'arguments': request['params']['arguments']}}))
                reply = await accepted
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': request['id'],
                    'result': {'accepted': True, 'job': job} if reply.get('accepted') else reply}))
                if reply.get('accepted'):
                    value = await result
                    await ws.send(json.dumps({'jsonrpc': '2.0', 'method': 'hub/result',
                        'params': {'job': job, 'device': 'skill-test', 'tool': request['params']['tool'],
                                   'time': time.time(), **value}}))
            finally:
                pending.pop(call, None)

        async def handler(ws):
            if ws.request.path == '/ws/mhs':
                register = json.loads(await ws.recv())
                assert not errors('RegisterParams', register['params'])
                connections['device'] = ws
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': register['id'], 'result': {
                    'session': 'test', 'hub': {'name': 'test', 'version': '1'}, 'time': time.time()}}))
                registered.set()
                async for raw in ws:
                    message = json.loads(raw)
                    if message.get('id') in pending:
                        pending[message['id']][0].set_result(message['result'])
                    elif message.get('method') == 'mhs/result':
                        value = message['params']
                        assert not errors('ResultParams', value)
                        if value['call'] in pending:
                            pending[value['call']][1].set_result({key: item for key, item in value.items() if key != 'call'})
            elif ws.request.path == '/ws/hub':
                async for raw in ws:
                    message = json.loads(raw)
                    if message['method'] == 'hub/hello':
                        await ws.send(json.dumps({'jsonrpc': '2.0', 'id': message['id'], 'result': {'session': 'agent'}}))
                    elif message['method'] == 'hub/call':
                        task = asyncio.create_task(forward(ws, message))
                        tasks.add(task)
                        task.add_done_callback(tasks.discard)
            elif ws.request.path == '/ws/nerve':
                async for raw in ws:
                    pass

        async with serve(handler, '127.0.0.1', 0, subprotocols=['mhs.v1'],
                         select_subprotocol=lambda conn, protocols: 'mhs.v1' if 'mhs.v1' in protocols else None) as server:
            url = f'ws://127.0.0.1:{server.sockets[0].getsockname()[1]}'
            device = SkillDevice(RobotCache(), 'skill-test',
                load_config(ROOT / 'ros2_ws/src/hei_agent_bridge/config/skills.mock.json'), None)
            assert device.snapshot()['valid'] and device.snapshot()['mode'] == 'SKILLS_READY'
            assert device.snapshot()['stop_scope'] == 'skill_owned_actions_only'
            worker = asyncio.create_task(device.device.run(url))
            pump = asyncio.create_task(device.pump(5))
            try:
                await asyncio.wait_for(registered.wait(), 5)
                async with HubClient(url) as hub:
                    connector = MHSConnector(hub, 'skill-test')
                    catalog = await connector.discover()
                    assert len(catalog.describe()) == 5
                    graph = Graph.load(ROOT / 'agent/src/hei_agent/examples/named_pose.json')
                    result = await Agent(replace(graph, catalog_revision=catalog.revision), connector).run({'side': 'right'})
                    assert result.status.value == 'succeeded'
                    grip = await connector.call('set_gripper_gap', {'side': 'right', 'gap_m': 0.025}, call_id='gripper')
                    assert grip.outputs['position_m'] == 0.025

                    graph_path = tmp_path / 'graph.json'
                    graph_path.write_text(json.dumps(graph.to_dict()))
                    command = [sys.executable, '-m', 'hei_agent', '--backend', 'mhs', '--hub-url', url,
                               '--device-id', 'skill-test', '--graph', str(graph_path), '--trace-dir', str(tmp_path)]
                    env = dict(os.environ)
                    env.pop('LD_PRELOAD', None)
                    env['PYTHONPATH'] = str(ROOT / 'agent/src') + os.pathsep + env.get('PYTHONPATH', '')
                    process = await asyncio.create_subprocess_exec(*command, env=env,
                        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
                    stdout, stderr = await asyncio.wait_for(process.communicate(), 10)
                    assert process.returncode == 0, stderr.decode()
                    cli_result = json.loads(stdout)
                    assert cli_result['status'] == 'succeeded' and cli_result['execution_backend'] == 'mock'

                    device.skills.backend.delay = 10
                    running = asyncio.create_task(connector.call('set_gripper_gap',
                        {'side': 'left', 'gap_m': 0.01}, call_id='cancelled'))
                    deadline = time.monotonic() + 3
                    while device.skills.motion_owner is None:
                        assert time.monotonic() < deadline
                        await asyncio.sleep(0.01)
                    running.cancel()
                    await asyncio.gather(running, return_exceptions=True)
                    await connector.cancel('cancelled')
                    assert device.skills.motion_owner is None
                    assert 'left' not in device.skills.backend.gaps
            finally:
                await device.stop()
                worker.cancel()
                pump.cancel()
                await asyncio.gather(worker, pump, return_exceptions=True)
                for task in list(tasks):
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)

    asyncio.run(exercise())
