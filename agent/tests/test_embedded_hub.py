"""Loopback-only embedded transport tests; no ROS or physical devices."""
import asyncio
import json

import pytest

pytest.importorskip('websockets')
from websockets.asyncio.client import connect

from hei_agent.hub import EmbeddedHub


async def register(hub, *, device_id='hei-01'):
    ws = await connect(f'ws://127.0.0.1:{hub.port}/ws/mhs', subprotocols=['mhs.v1'])
    await ws.send(json.dumps({'jsonrpc': '2.0', 'id': 'register', 'method': 'mhs/register', 'params': {
        'protocol': 'mhs/v1', 'device': {'id': device_id}, 'tools': [{'name': 'test'}]}}))
    reply = json.loads(await ws.recv())
    return ws, reply


def test_wait_registration_and_final_result():
    async def exercise():
        async with EmbeddedHub('127.0.0.1', 0) as hub:
            waiter = asyncio.create_task(hub.wait_device('hei-01', 1))
            ws, reply = await register(hub)
            try:
                assert 'result' in reply
                await waiter
                task = asyncio.create_task(hub.invoke('hei-01', 'test', {}))
                call = json.loads(await ws.recv())
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': call['id'], 'result': {'accepted': True}}))
                await asyncio.sleep(.01)
                assert not task.done()  # ACK is not execution completion.
                await ws.send(json.dumps({'jsonrpc': '2.0', 'method': 'mhs/result', 'params': {
                    'call': call['id'], 'status': 'done', 'data': {'value': 42}}}))
                assert (await task)['data'] == {'value': 42}
                assert not hub.devices['hei-01'].pending
            finally:
                await ws.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('failure', ['rejected', 'rpc_error', 'disconnect', 'timeout', 'cancel'])
def test_pending_cleanup(failure):
    async def exercise():
        async with EmbeddedHub('127.0.0.1', 0) as hub:
            ws, _ = await register(hub)
            device = hub.devices['hei-01']
            task = asyncio.create_task(hub.invoke('hei-01', 'test', {}, timeout_sec=.1))
            call = json.loads(await ws.recv())
            if failure == 'rejected':
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': call['id'], 'result': {'accepted': False}}))
            elif failure == 'rpc_error':
                await ws.send(json.dumps({'jsonrpc': '2.0', 'id': call['id'], 'error': {'code': -32602}}))
            elif failure == 'disconnect':
                await ws.close()
            elif failure == 'cancel':
                task.cancel()
            with pytest.raises((RuntimeError, asyncio.TimeoutError, asyncio.CancelledError)):
                await task
            assert not device.pending
            await ws.close()
    asyncio.run(exercise())


def test_unexpected_and_duplicate_devices_cannot_replace_connection():
    async def exercise():
        async with EmbeddedHub('127.0.0.1', 0) as hub:
            first, _ = await register(hub)
            original = hub.devices['hei-01']
            for identity in ['hei-01', 'other']:
                ws, reply = await register(hub, device_id=identity)
                assert 'error' in reply
                await ws.close()
            assert hub.devices['hei-01'] is original
            await first.close()
    asyncio.run(exercise())


def test_wait_timeout_and_unpublished_tools():
    async def exercise():
        async with EmbeddedHub('127.0.0.1', 0) as hub:
            with pytest.raises(RuntimeError, match='Timed out'):
                await hub.wait_device('hei-01', .01)
            with pytest.raises(RuntimeError, match='not connected'):
                await hub.invoke('hei-01', 'test', {})
            ws, _ = await register(hub)
            with pytest.raises(ValueError, match='does not publish'):
                await hub.invoke('hei-01', 'missing', {})
            await ws.close()
    asyncio.run(exercise())
