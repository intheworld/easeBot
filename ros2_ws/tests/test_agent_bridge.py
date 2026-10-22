"""Real MHS SDK over WebSockets plus a real mock ROS bringup; no physical hardware."""
import asyncio
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import CompressedImage, JointState
from hei_agent_bridge.cache import RobotCache


def test_feedback_and_images_cannot_be_refreshed_by_duplicate_or_invalid_messages():
    cache = RobotCache()
    joint = JointState(name=['lift_joint'], position=[-0.2])
    joint.header.stamp.sec = 1
    cache.joints(joint)
    cache.last_joint = time.monotonic() - 3
    cache.joints(joint)
    assert not cache.snapshot()['joint_valid']
    assert cache.snapshot()['mode'] == 'UNKNOWN'
    invalid = JointState(name=['x', 'x'], position=[0.0, 1.0])
    invalid.header.stamp.sec = 2
    cache.joints(invalid)
    assert not cache.snapshot()['joint_valid']
    ok, jpeg = cv2.imencode('.jpg', np.zeros((48, 64, 3), dtype=np.uint8))
    assert ok
    image = CompressedImage(format='jpeg', data=jpeg.tobytes())
    image.header.stamp.sec = 1
    cache.image('front', image)
    assert cache.frame('front').width == 64
    cache.frames['front'] = replace(cache.frames['front'], received=time.monotonic() - 3)
    cache.image('front', image)
    assert cache.frame('front') is None
    cache.image('left_wrist', CompressedImage(format='jpeg', data=b'broken'))
    assert cache.frame('left_wrist') is None


def test_mhs_registration_readonly_tools_images_reconnect_and_shutdown(monkeypatch):
    pytest.importorskip('agnes_mhs', reason='source scripts/env/agent-bridge.bash')
    from agnes_mhs import errors
    from websockets.asyncio.server import serve
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState

    domain = str(180 + os.getpid() % 30)
    monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    rclpy.init(args=[])
    node = rclpy.create_node('agent_bridge_acceptance')

    async def exercise():
        registrations, messages, pictures = [], [], []
        connections = {}
        async def hub(ws):
            if ws.request.path == '/ws/mhs':
                register = json.loads(await ws.recv())
                assert not errors('RegisterParams', register['params'])
                registrations.append(register['params'])
                connections['command'] = ws
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': register['id'], 'result': {
                    'session': str(len(registrations)), 'hub': {'name': 'HEI test hub', 'version': '1'},
                    'time': time.time()}}))
                async for raw in ws:
                    msg = json.loads(raw)
                    if msg.get('method') == 'mhs/state':
                        assert not errors('StateParams', msg['params'])
                    if msg.get('method') == 'mhs/result':
                        assert not errors('ResultParams', msg['params'])
                    messages.append(msg)
            elif ws.request.path == '/ws/nerve':
                hello = json.loads(await ws.recv())
                assert hello == {'type': 'hello', 'device': 'hei-test-01'}
                metadata = None
                async for raw in ws:
                    if isinstance(raw, bytes):
                        assert metadata is not None
                        assert metadata['source'] in ('front', 'left_wrist', 'right_wrist')
                        assert not errors('ImageData', metadata['data'])
                        assert raw.startswith(b'\xff\xd8')
                        assert cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR) is not None
                        pictures.append((metadata, raw))
                        metadata = None
                    else:
                        metadata = json.loads(raw)
                        assert not errors('NerveData', metadata)
                        if not metadata.get('bin'):
                            metadata = None

        async with serve(hub, '127.0.0.1', 0, subprotocols=['mhs.v1']) as server:
            port = server.sockets[0].getsockname()[1]
            with tempfile.TemporaryDirectory(prefix='hei-agent-') as temp, tempfile.TemporaryFile(mode='w+') as log:
                env = {**os.environ, 'ROS_LOG_DIR': temp, 'NO_PROXY': '127.0.0.1,localhost'}
                env.pop('LD_PRELOAD', None)  # Do not inject the tool host's network proxy into ROS C++ nodes.
                launch = subprocess.Popen(['ros2', 'launch', 'hei_bringup', 'robot.launch.py',
                    'monitor:=false', 'agent_bridge:=true', f'agent_hub_url:=ws://127.0.0.1:{port}',
                    'agent_device_id:=hei-test-01'], env=env, stdout=log, stderr=log, start_new_session=True)
                async def until(predicate, timeout=20):
                    deadline = time.monotonic() + timeout
                    while time.monotonic() < deadline:
                        assert launch.poll() is None, 'bringup exited'
                        rclpy.spin_once(node, timeout_sec=0)
                        if predicate():
                            return
                        await asyncio.sleep(0.02)
                    raise AssertionError('timed out waiting for MHS/ROS')
                async def request(id, method, params):
                    await connections['command'].send(json.dumps({'jsonrpc': '2.0', 'id': id, 'method': method, 'params': params}))
                    await until(lambda: any(m.get('id') == id for m in messages))
                    return next(m for m in messages if m.get('id') == id)
                async def snapshot(id):
                    accepted = await request(id, 'mhs/call', {'name': 'get_robot_state', 'arguments': {}})
                    assert accepted['result']['accepted']
                    await until(lambda: any(m.get('method') == 'mhs/result' and m['params']['call'] == id for m in messages))
                    return next(m['params']['data'] for m in messages if m.get('method') == 'mhs/result' and m['params']['call'] == id)
                try:
                    await until(lambda: registrations and any(m.get('method') == 'mhs/state' and m['params']['values'].get('valid') for m in messages)
                                and any(m.get('method') == 'mhs/state' and m['params']['values'].get('mode') == 'DISABLED' for m in messages)
                                and len({m['source'] for m, _ in pictures}) == 3)
                    assert registrations[0]['tools'][0]['readOnly']
                    assert not any(t.get('motion') for t in registrations[0]['tools'])
                    await until(lambda: ('agent_bridge', '/hei') in node.get_node_names_and_namespaces())
                    assert node.get_client_names_and_types_by_node('agent_bridge', '/hei') == []
                    assert {n for n, _ in node.get_publisher_names_and_types_by_node('agent_bridge', '/hei')}.issubset({'/rosout', '/parameter_events'})
                    data = await snapshot('read-1')
                    assert data['valid'] and data['mode'] == 'DISABLED'
                    assert len(data['positions']) == 21
                    assert all(c['valid'] for c in data['cameras'].values())
                    assert data['backend'] == 'ros2_control_mock'
                    assert (await request('motion-1', 'mhs/call', {'name': 'move', 'arguments': {}}))['result']['accepted'] is False
                    assert (await request('stop-1', 'mhs/stop', {}))['result']['stopped'] == []
                    assert (await snapshot('read-2'))['mode'] == 'DISABLED'
                    # Exercise SDK command reconnect without restarting ROS or changing identity.
                    await connections['command'].close(code=1011, reason='test reconnect')
                    await until(lambda: len(registrations) >= 2)
                    assert (await snapshot('read-3'))['valid']
                    change = node.create_client(ChangeState, '/hei/cameras/change_state')
                    await until(lambda: change.service_is_ready())
                    req = ChangeState.Request()
                    req.transition.id = Transition.TRANSITION_DEACTIVATE
                    future = change.call_async(req)
                    await until(future.done)
                    assert future.result().success
                    await asyncio.sleep(1.3)
                    assert all(not c['valid'] for c in (await snapshot('read-4'))['cameras'].values())
                    count = len(pictures)
                    await asyncio.sleep(1.1)
                    assert len(pictures) == count, 'stale JPEG was retransmitted'
                finally:
                    if launch.poll() is None:
                        os.killpg(launch.pid, signal.SIGINT)
                    try:
                        await asyncio.to_thread(launch.wait, timeout=12)
                    except subprocess.TimeoutExpired:
                        os.killpg(launch.pid, signal.SIGKILL)
                        await asyncio.to_thread(launch.wait)
                        pytest.fail('bringup/bridge failed to exit')
                    log.seek(0)
                    output = log.read()
                    print(output[-12000:])
                    assert any(line.startswith('[INFO] [agent_bridge-') and 'process has finished cleanly' in line for line in output.splitlines())
    try:
        asyncio.run(exercise())
    finally:
        node.destroy_node()
        rclpy.shutdown()
