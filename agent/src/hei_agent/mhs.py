"""AgnesHub northbound adapter. No reconnect, action replay or acceptance-as-success."""
import asyncio
import base64
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import urlsplit

from .catalog import SkillCatalog
from .graph import json_data, positive_seconds
from .node import NodeResult


class HubClient:
    def __init__(self, url, *, timeout_sec=5.0):
        positive_seconds(timeout_sec, 'Hub RPC timeout')
        parsed = urlsplit(url)
        if (parsed.scheme not in ('ws', 'wss') or not parsed.hostname or parsed.username or parsed.password
                or parsed.path not in ('', '/') or parsed.query or parsed.fragment):
            raise ValueError('Hub URL must be a ws(s) base origin without credentials or /ws/hub')
        self.url, self.timeout_sec = url.rstrip('/') + '/ws/hub', timeout_sec
        self.ws, self.reader = None, None
        self.pending, self.results, self.jobs = {}, {}, {}
        self.sequence = 0

    async def __aenter__(self):
        try:
            from websockets.asyncio.client import connect
        except ImportError as exc:
            raise RuntimeError('Prepare server dependencies: bash scripts/setup.bash --agent-server') from exc
        self.ws = await connect(self.url, open_timeout=self.timeout_sec, max_size=4 * 1024 * 1024)
        self.reader = asyncio.create_task(self._read())
        try:
            await self.rpc('hub/hello', {'role': 'agent', 'client': {'name': 'hei-agent', 'version': '0.1.0'}})
        except BaseException:
            await self.__aexit__(None, None, None)
            raise
        return self

    async def __aexit__(self, *args):
        if self.ws is not None:
            await self.ws.close()
        if self.reader is not None:
            self.reader.cancel()
            await asyncio.gather(self.reader, return_exceptions=True)

    async def _read(self):
        try:
            async for raw in self.ws:
                if not isinstance(raw, str):
                    continue
                message = json.loads(raw)
                if 'id' in message:
                    future = self.pending.get(str(message['id']))
                    if future is not None and not future.done():
                        if 'error' in message:
                            future.set_exception(RuntimeError('AgnesHub rejected the RPC request'))
                        else:
                            future.set_result(message.get('result'))
                elif message.get('method') == 'hub/result':
                    result = message['params']
                    job = result['job']
                    # Bounded early-result cache covers a result arriving immediately
                    # after acceptance, before the waiting coroutine resumes.
                    if job not in self.results and len(self.results) >= 256:
                        self.results.pop(next(iter(self.results)))
                    self.results[job] = result
                    future = self.jobs.get(job)
                    if future is not None and not future.done():
                        future.set_result(result)
        except asyncio.CancelledError:
            raise
        except Exception:
            pass
        finally:
            for future in list(self.pending.values()) + list(self.jobs.values()):
                if not future.done():
                    future.set_exception(RuntimeError('AgnesHub disconnected; execution state may be unknown'))

    async def rpc(self, method, params):
        if self.reader is None or self.reader.done():
            raise RuntimeError('AgnesHub is not connected')
        self.sequence += 1
        request = str(self.sequence)
        future = asyncio.get_running_loop().create_future()
        future.add_done_callback(lambda done: None if done.cancelled() else done.exception())
        self.pending[request] = future
        try:
            await self.ws.send(json.dumps({'jsonrpc': '2.0', 'id': request, 'method': method, 'params': params}))
            return await asyncio.wait_for(asyncio.shield(future), self.timeout_sec)
        finally:
            self.pending.pop(request, None)

    async def wait_job(self, job):
        if job in self.results:
            return self.results.pop(job)
        if self.reader is None or self.reader.done():
            raise RuntimeError('AgnesHub disconnected while waiting for job')
        future = asyncio.get_running_loop().create_future()
        future.add_done_callback(lambda done: None if done.cancelled() else done.exception())
        self.jobs[job] = future
        try:
            result = await future
            self.results.pop(job, None)
            return result
        finally:
            self.jobs.pop(job, None)

    async def invoke(self, device, tool, arguments, *, ref=None, timeout_sec=5.0):
        reply = await self.rpc('hub/call', {'device': device, 'tool': tool, 'arguments': arguments,
                                           **({'ref': ref} if ref else {})})
        if not isinstance(reply, dict) or not reply.get('accepted'):
            reason = reply.get('reason', 'unknown') if isinstance(reply, dict) else 'malformed reply'
            raise RuntimeError(f'AgnesHub/device rejected skill call ({str(reason)[:100]})')
        return await asyncio.wait_for(self.wait_job(reply['job']), timeout_sec)


def result_data(result):
    if result.get('status') != 'done' or type(result.get('data')) is not dict:
        raise RuntimeError('Remote skill did not return a completed result')
    return result['data']


class MHSConnector:
    def __init__(self, hub, device_id, *, artifact_dir='log/hei_agent/observations'):
        self.hub, self.device_id = hub, device_id
        self.catalog = None
        self.invocations = {}
        self.artifact_dir = Path(artifact_dir)

    async def discover(self):
        result = await self.hub.invoke(self.device_id, 'get_skill_catalog', {})
        catalog = SkillCatalog.from_manifest(result_data(result))
        self.catalog = catalog
        return catalog

    async def call(self, skill, arguments, *, call_id):
        if self.catalog is None:
            raise RuntimeError('Discover the robot skill catalog first')
        if call_id in self.invocations:
            raise RuntimeError('Call ID already used; no automatic replay')
        spec = self.catalog.get(skill)
        spec.validate_arguments(arguments)
        invocation = hashlib.sha256(call_id.encode()).hexdigest()
        task = asyncio.create_task(self.hub.invoke(self.device_id, skill,
            {**json_data(arguments), 'revision': self.catalog.revision, 'invocation_id': invocation},
            ref=call_id, timeout_sec=spec.timeout_sec + 5))
        task.add_done_callback(lambda done: None if done.cancelled() else done.exception())
        self.invocations[call_id] = (invocation, task)
        result = result_data(await asyncio.shield(task))
        if (result.get('invocation_id') != invocation or result.get('revision') != self.catalog.revision
                or result.get('cleanup_complete') is not True):
            raise RuntimeError('Remote skill identity or termination unconfirmed')
        outcome, outputs = result.get('outcome'), result.get('outputs', {})
        if outcome not in spec.outcomes:
            raise RuntimeError('Unexpected remote skill outcome')
        if outcome == 'success':
            spec.validate_outputs(outputs)
            if skill == 'get_camera_observation':
                outputs = await self._observation_assets(outputs)
        return NodeResult(outcome, json_data(outputs), result.get('message', '')).validate()

    async def _observation_assets(self, outputs):
        observation_id = outputs.get('observation_id', '')
        if not re.fullmatch(r'[a-f0-9]{32}', observation_id):
            raise RuntimeError('Invalid observation identity')
        assets = []
        for kind, mime, suffix in (('rgb', 'image/jpeg', '.jpg'), ('depth', 'application/x-npy', '.npy'),
                                   ('metadata', 'application/json', '.json')):
            reference = outputs.get(kind + '_ref')
            if not reference and kind == 'depth' and outputs.get('depth_available') is False:
                continue
            if reference != f'mhs-observation:{observation_id}/{kind}':
                raise RuntimeError('Unexpected observation asset reference')
            value = result_data(await self.hub.invoke(self.device_id, 'read_observation_asset',
                {'observation_id': observation_id, 'kind': kind}, timeout_sec=5))
            encoded = value.get('data_base64')
            if (value.get('mime_type') != mime or type(encoded) is not str
                    or len(encoded) > 2_800_000 or type(value.get('byte_length')) is not int):
                raise RuntimeError('Invalid observation asset envelope')
            data = base64.b64decode(encoded, validate=True)
            if (len(data) != value['byte_length'] or not 0 < len(data) <= 2 * 1024 * 1024
                    or hashlib.sha256(data).hexdigest() != value.get('sha256')):
                raise RuntimeError('Observation asset integrity check failed')
            assets.append((kind, suffix, data))
        # Download/verify everything before persisting. Only local files are
        # returned; no remote path/URL is ever opened. NPY is stored, not loaded.
        directory = self.artifact_dir.resolve() / observation_id
        directory.mkdir(parents=True, exist_ok=True)
        outputs = dict(outputs)
        for kind, suffix, data in assets:
            destination = directory / (kind + suffix)
            temporary = directory / (kind + suffix + '.part')
            try:
                temporary.write_bytes(data)
                temporary.replace(destination)
            finally:
                temporary.unlink(missing_ok=True)
            outputs[kind + '_ref'] = str(destination)
        return outputs

    async def cancel(self, call_id):
        record = self.invocations.get(call_id)
        if record is None:
            # No transport dispatch has happened for this local ID.
            return
        invocation, task = record
        result = await self.hub.invoke(self.device_id, 'cancel_skill', {'invocation_id': invocation}, timeout_sec=5)
        data = result_data(result)
        if data.get('cleanup_complete') is not True or data.get('status') not in (
                'not_started', 'succeeded', 'failed', 'cancelled'):
            raise RuntimeError('Robot did not confirm skill termination')
        # The tombstone on the robot prevents a late dispatch from starting.
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
