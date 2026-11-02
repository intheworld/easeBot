"""Shared robot motion-plan lifetime, compute cleanup and scene monitoring."""
import asyncio
from collections import OrderedDict
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
import uuid
import xml.etree.ElementTree as ET
from .skills import digest


class MotionComputeError(RuntimeError):
    def __init__(self,message,error_code=None):
        super().__init__(message)
        self.error_code=error_code


def data_directory():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('hei_agent_bridge')) / 'config'
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[2] / 'config'


def validate_compute_config(profile):
    for field in ('startup_warmup','persistent_worker'):
        if field in profile and type(profile[field]) is not bool:
            raise ValueError(field+' must be a boolean')
    timeout=profile.get('worker_idle_timeout_sec',0)
    if type(timeout) not in (int,float) or not math.isfinite(timeout) or timeout<0:
        raise ValueError('worker_idle_timeout_sec must be finite and nonnegative')


class MotionRuntime:
    def __init__(self, profile, backend, *, directory, worker_script, urdf=None,
                 extra_files=(), worker_cwd=None):
        self.profile, self.backend = deepcopy(profile), backend
        self.directory = Path(directory).resolve()
        if urdf is None:
            from ament_index_python.packages import get_package_share_directory
            urdf = Path(get_package_share_directory('hei_description')) / 'urdf/hei_body.urdf'
        self.urdf = str(Path(urdf).resolve(strict=True))
        names = {Path(e.get('filename')).name for e in ET.parse(self.urdf).getroot().iter('mesh')}
        self.meshes = sorted(str((Path(self.urdf).parent.parent/'meshes'/n).resolve(strict=True)) for n in names)
        self.extra_files = tuple(str(Path(p).resolve(strict=True)) for p in extra_files)
        self.worker_script = str(Path(worker_script).resolve(strict=True))
        self.worker_cwd = str(Path(worker_cwd or Path.cwd()).resolve(strict=True))
        self.plans, self.computing = OrderedDict(), False
        self._file_hashes = {}
        self.binding = self._binding()
        self._compute_lock = asyncio.Lock()
        self._compute_process = None
        self.warmup_status={"status":"not_started"}

    def _binding(self):
        files = {}
        for name in (self.urdf, *self.extra_files, *self.meshes):
            path = Path(name)
            stamp = lambda s: (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns,s.st_mode)
            key = stamp(path.stat())
            cached = self._file_hashes.get(name)
            if cached is None or cached[0] != key:
                value = hashlib.sha256(path.read_bytes()).hexdigest()
                if stamp(path.stat()) != key:
                    raise RuntimeError('Motion model changed while computing its binding')
                self._file_hashes[name] = (key,value)
            files[name] = self._file_hashes[name][1]
        return digest({'profile':self.profile,'files':files})

    @staticmethod
    def scene_matches(before, after):
        return (abs(before['lift']-after['lift']) <= .005 and
                max(abs(a-b) for a,b in zip(before['wheels'], after['wheels'])) <= .02)

    def same_start(self, before, after):
        return (self.scene_matches(before, after) and
                max(abs(a-b) for a,b in zip(before['right'], after['right'])) <= .02 and
                abs(before['right_gripper']-after['right_gripper']) <= .02)

    def remember(self, plan_id, data, state):
        while len(self.plans) >= 4:
            self.plans.popitem(last=False)
        self.plans[plan_id] = {'data': data, 'state': state, 'created': time.monotonic(), 'consumed': False}

    def checked_plan(self, plan_id):
        if not re.fullmatch('[0-9a-f]{32}', plan_id):
            raise ValueError('Invalid motion plan ID')
        entry = self.plans.get(plan_id)
        if entry is None or entry['consumed'] or time.monotonic()-entry['created'] > 60:
            raise RuntimeError('Motion plan missing, expired or already consumed; replan')
        if entry['data']['validation']['status'] != 'passed':
            raise RuntimeError('Motion collision/scene validation did not pass')
        if self._binding() != self.binding:
            raise RuntimeError('Motion model/configuration changed; restart and replan')
        return entry

    async def guarded(self, run, matches, *, message):
        async def watch():
            while True:
                if not matches(self.state(at_rest=False)):
                    raise RuntimeError(message)
                await asyncio.sleep(.05)
        motion, guard = asyncio.create_task(run()), asyncio.create_task(watch())
        try:
            await asyncio.wait({motion, guard}, return_when=asyncio.FIRST_COMPLETED)
            if guard.done():
                guard.result()
            return motion.result()
        finally:
            motion.cancel(); guard.cancel()
            await asyncio.gather(motion, guard, return_exceptions=True)

    async def warmup(self):
        """Queue synthetic compute on the same lock/process as real requests."""
        if not self.profile.get('startup_warmup',True) or not self.profile.get('persistent_worker',True):
            self.warmup_status={'status':'disabled'}
            return self.warmup_status
        directory=self.directory/('_warmup_'+uuid.uuid4().hex)
        self.warmup_status={'status':'warming','directory':str(directory)}
        try:
            directory.mkdir(parents=True)
            (directory/'request.json').write_text(json.dumps(dict(operation='warmup',urdf=self.urdf,
                                                                profile=self.profile),allow_nan=False))
            result=await self._worker(directory)
            if result.get('operation')!='warmup' or result.get('status')!='ready':
                raise RuntimeError('Invalid compute warmup response')
            self.warmup_status={**result,'directory':str(directory),'pid':self._compute_process.pid}
            return self.warmup_status
        except asyncio.CancelledError:
            self.warmup_status={'status':'cancelled','directory':str(directory)}
            raise
        except Exception as exc:
            self.warmup_status={'status':'failed','directory':str(directory),'message':str(exc)}
            raise

    async def close_worker(self):
        process, self._compute_process = self._compute_process, None
        if process is not None:
            if process.returncode is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(process.wait(), 1)
                except asyncio.TimeoutError:
                    process.kill()
                    await process.wait()
            else:
                await process.wait()

    async def _worker(self, directory):
        if not self.profile.get('persistent_worker', True):
            return await self._single_worker(directory)
        async with self._compute_lock:
            directory = Path(directory).resolve()
            result = directory/'result.json'
            result.unlink(missing_ok=True)
            env = dict(os.environ)
            env['HEI_COMPUTE_IDLE_TIMEOUT_SEC']=str(self.profile.get('worker_idle_timeout_sec',0))
            for key in ('PYTHONHOME', 'PYTHONPATH', 'PYTHONNOUSERSITE'):
                env.pop(key, None)
            try:
                if self._compute_process is None or self._compute_process.returncode is not None:
                    await self.close_worker()
                    with (directory/'worker-startup.log').open('ab') as log:
                        spawning = asyncio.create_task(asyncio.create_subprocess_exec(
                            self.profile['worker_python'], self.worker_script, '--serve',
                            env=env, cwd=self.worker_cwd, stdin=asyncio.subprocess.PIPE,
                            stdout=asyncio.subprocess.PIPE, stderr=log))
                        try:
                            self._compute_process = await asyncio.shield(spawning)
                        except asyncio.CancelledError:
                            self._compute_process = await spawning
                            raise
                process = self._compute_process
                request_id = uuid.uuid4().hex
                process.stdin.write((json.dumps({'id':request_id, 'directory':str(directory)})+'\n').encode())
                await process.stdin.drain()
                reply = json.loads(await asyncio.wait_for(process.stdout.readline(), 180))
                if reply.get('id') != request_id:
                    raise RuntimeError('Motion worker response ID mismatch')
                (directory/'worker-timing.json').write_text(json.dumps(reply))
                if not result.is_file() or result.stat().st_size > 2*1024*1024:
                    raise RuntimeError(f'Motion compute failed; see {directory / "worker.log"}')
                data = json.loads(result.read_text())
                if reply['status'] != 0 or 'error' in data:
                    raise MotionComputeError(data.get('error', 'Motion worker failed'), data.get('error_code'))
                return data
            except MotionComputeError:
                # Expected observation/planning failures do not discard warm models.
                raise
            except BaseException:
                # Never allow a cancelled request to finish in the background or
                # have its result consumed by a later invocation.
                await self.close_worker()
                raise

    async def _single_worker(self, directory):
        result=directory/'result.json'
        result.unlink(missing_ok=True)  # Every retry must consume a newly produced result.
        env = dict(os.environ)
        env.pop('PYTHONHOME', None)
        env.pop('PYTHONPATH', None)
        env.pop('PYTHONNOUSERSITE', None)  # ROS isolation must not hide the compute environment's Torch.
        process = None
        with (directory / 'worker.log').open('wb') as log:
            try:
                spawning = asyncio.create_task(asyncio.create_subprocess_exec(self.profile['worker_python'],
                    self.worker_script, str(directory),
                    env=env, cwd=self.worker_cwd, stdout=log, stderr=log))
                try:
                    process = await asyncio.shield(spawning)
                except asyncio.CancelledError:
                    # Cancellation racing with spawn must still reap the child.
                    process = await spawning
                    raise
                await process.wait()
            finally:
                if process is not None and process.returncode is None:
                    process.terminate()
                    try:
                        await asyncio.wait_for(process.wait(), 1)
                    except asyncio.TimeoutError:
                        process.kill()
                        await process.wait()
        result = directory / 'result.json'
        if not result.is_file() or result.stat().st_size > 2 * 1024 * 1024:
            raise RuntimeError(f'Motion compute failed; see {directory / "worker.log"}')
        data = json.loads(result.read_text())
        if process.returncode != 0 or 'error' in data:
            raise MotionComputeError(data.get('error', 'Motion worker failed'),data.get('error_code'))
        return data

