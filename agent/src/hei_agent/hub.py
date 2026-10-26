"""In-process MHS skill transport, not a replacement for the full AgnesHub API.

Only robot registration, state and call results are consumed. Nerve frames are
drained without storage or interpretation. No reconnect/replay or motion retry.
"""
import asyncio
from dataclasses import dataclass, field
import json
import re
import time
import uuid

from .graph import positive_seconds


@dataclass
class _Device:
    ws: object
    tools: set
    pending: dict = field(default_factory=dict)
    state: dict = field(default_factory=dict)


class EmbeddedHub:
    def __init__(self, host='0.0.0.0', port=4180, *, device_id='hei-01'):
        if not re.fullmatch(r'[a-z0-9][a-z0-9-]{0,31}', device_id):
            raise ValueError('Invalid MHS device ID')
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError('Listen port must be between 0 and 65535')
        self.host, self.port, self.device_id = host, port, device_id
        self.server = None
        self.devices = {}
        self.changed = asyncio.Event()

    async def __aenter__(self):
        try:
            from websockets.asyncio.server import serve
        except ImportError as exc:
            raise RuntimeError('Prepare server dependencies: bash scripts/setup.bash --agent-server') from exc
        self.server = await serve(self._handle, self.host, self.port,
            subprotocols=['mhs.v1'], ping_interval=2, ping_timeout=3,
            max_size=16 * 1024 * 1024, max_queue=4)
        self.port = self.server.sockets[0].getsockname()[1]
        return self

    async def __aexit__(self, *args):
        # Connection loss triggers the SDK's on_stop; it isn't proof of cleanup.
        # Agent/connector must obtain explicit cancellation confirmation first.
        self.server.close()
        await self.server.wait_closed()

    async def wait_device(self, device_id, timeout_sec=30):
        positive_seconds(timeout_sec, 'Device wait timeout')

        async def wait():
            while device_id not in self.devices:
                self.changed.clear()
                await self.changed.wait()

        try:
            await asyncio.wait_for(wait(), timeout_sec)
        except asyncio.TimeoutError as exc:
            raise RuntimeError(f'Timed out waiting for MHS device {device_id}') from exc

    @staticmethod
    def _message(raw):
        if not isinstance(raw, str):
            raise ValueError('MHS command must be JSON text')
        message = json.loads(raw)
        if type(message) is not dict or message.get('jsonrpc') != '2.0':
            raise ValueError('Invalid JSON-RPC message')
        return message

    async def _handle(self, ws):
        from websockets.exceptions import ConnectionClosed
        try:
            if ws.request.path == '/ws/nerve':
                hello = json.loads(await asyncio.wait_for(ws.recv(), 5))
                if (type(hello) is not dict or hello.get('type') != 'hello'
                        or hello.get('device') != self.device_id):
                    raise ValueError('Unexpected Nerve device')
                async for _ in ws:
                    pass  # Images/telemetry are not exposed to policy nodes yet.
            elif ws.request.path == '/ws/mhs':
                await self._command(ws)
            else:
                await ws.close(1008, 'Unsupported endpoint')
        except (ValueError, TypeError, KeyError, asyncio.TimeoutError):
            await ws.close(1008, 'Invalid MHS message')
        except ConnectionClosed:
            pass

    async def _command(self, ws):
        request = self._message(await asyncio.wait_for(ws.recv(), 5))
        params = request.get('params')
        if (request.get('method') != 'mhs/register' or type(params) is not dict
                or 'id' not in request):
            raise ValueError('Registration required')
        info, declarations = params.get('device'), params.get('tools', [])
        valid = (params.get('protocol') == 'mhs/v1' and type(info) is dict
                 and info.get('id') == self.device_id and type(declarations) is list
                 and all(type(tool) is dict and type(tool.get('name')) is str for tool in declarations))
        device_id = info.get('id') if type(info) is dict else None
        if not valid or device_id in self.devices:
            await ws.send(json.dumps({'jsonrpc': '2.0', 'id': request['id'], 'error': {
                'code': -32602, 'message': 'Unexpected, duplicate or invalid device registration'}}))
            await ws.close(1008, 'Registration refused')
            return
        device = _Device(ws, {tool['name'] for tool in declarations})
        self.devices[device_id] = device
        try:
            await ws.send(json.dumps({'jsonrpc': '2.0', 'id': request['id'], 'result': {
                'session': uuid.uuid4().hex, 'hub': {'name': 'hei-agent', 'version': '0.1.0'},
                'time': time.time()}}))
            self.changed.set()
            async for raw in ws:
                message = self._message(raw)
                if 'id' in message and 'method' not in message:
                    pending = device.pending.get(str(message['id']))
                    if pending is not None and not pending[0].done():
                        if 'error' in message:
                            pending[0].set_exception(RuntimeError('Device rejected MHS RPC'))
                        else:
                            pending[0].set_result(message.get('result'))
                elif message.get('method') == 'mhs/result':
                    value = message.get('params')
                    if (type(value) is not dict or type(value.get('call')) is not str
                            or value.get('status') not in ('done', 'error', 'interrupted', 'rejected')):
                        raise ValueError('Invalid call result')
                    pending = device.pending.get(value['call'])
                    if pending is not None and not pending[1].done():
                        pending[1].set_result(value)
                elif message.get('method') == 'mhs/state':
                    params = message.get('params')
                    value = params.get('values') if type(params) is dict else None
                    if type(value) is not dict:
                        raise ValueError('Invalid state update')
                    # Bound storage even when peers send undeclared field names.
                    device.state.update(value)
                    if len(device.state) > 256:
                        raise ValueError('Too many state fields')
        finally:
            self.devices.pop(device_id, None)
            for ack, result in device.pending.values():
                for future in (ack, result):
                    if not future.done():
                        future.set_exception(RuntimeError('MHS device disconnected; execution state unknown'))
            self.changed.set()

    async def invoke(self, device_id, tool, arguments, *, ref=None, timeout_sec=5):
        """Same connector-facing API as HubClient, without a northbound socket."""
        positive_seconds(timeout_sec, 'MHS call timeout')
        device = self.devices.get(device_id)
        if device is None:
            raise RuntimeError(f'MHS device {device_id} is not connected')
        if tool not in device.tools:
            raise ValueError(f'Device does not publish tool {tool}')
        if len(device.pending) >= 128:
            raise RuntimeError('Too many pending MHS calls')
        call = uuid.uuid4().hex
        loop = asyncio.get_running_loop()
        ack, result = loop.create_future(), loop.create_future()
        device.pending[call] = ack, result

        async def execute():
            await device.ws.send(json.dumps({'jsonrpc': '2.0', 'id': call, 'method': 'mhs/call',
                'params': {'name': tool, 'arguments': arguments}}, allow_nan=False))
            accepted = await asyncio.shield(ack)
            if type(accepted) is not dict or accepted.get('accepted') is not True:
                raise RuntimeError('MHS device did not accept the call')
            return await asyncio.shield(result)

        try:
            return await asyncio.wait_for(execute(), timeout_sec)
        finally:
            device.pending.pop(call, None)
            for future in (ack, result):
                if not future.done():
                    future.cancel()
                elif not future.cancelled():
                    future.exception()  # Consume a simultaneous disconnect failure.
