"""Robot-owned skill set: one definition drives discovery, checks and dispatch.

No ROS or MHS import: backends and transport are separate adapters.
"""
import asyncio
from collections import OrderedDict
from copy import deepcopy
import hashlib
import json
import math
import re
from pathlib import Path


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def scalar(value, spec):
    types = {'string': type(value) is str, 'boolean': type(value) is bool,
             'number': type(value) in (int, float) and math.isfinite(value)}
    if not types.get(spec['type'], False):
        raise ValueError(f'Expected {spec["type"]}')
    if 'enum' in spec and value not in spec['enum']:
        raise ValueError('Value not in declared enum')
    if spec['type'] == 'number' and (value < spec.get('minimum', -math.inf) or value > spec.get('maximum', math.inf)):
        raise ValueError('Value outside declared range')


def fields(values, definitions):
    if type(values) is not dict or set(values) != set(definitions):
        raise ValueError(f'Expected exactly these fields: {sorted(definitions)}')
    for key, spec in definitions.items():
        scalar(values[key], spec)


def load_config(path):
    config = json.loads(Path(path).read_text())
    if (type(config) is not dict or not {'backend', 'named_poses', 'gripper_max_gap_m'} <= set(config)
            or set(config) - {'backend', 'named_poses', 'gripper_max_gap_m', 'right_grasp', 'arm_motion'}):
        raise ValueError('Skill config requires backend, named_poses and gripper_max_gap_m')
    if config['backend'] not in ('mock', 'ros'):
        raise ValueError('Skill backend must be mock or ros')
    if 'right_grasp' in config and (config['backend'] != 'ros' or type(config['right_grasp']) is not dict):
        raise ValueError('right_grasp requires ros backend and an object of profile overrides')
    if 'arm_motion' in config and (config['backend'] != 'ros' or type(config['arm_motion']) is not dict):
        raise ValueError('arm_motion requires ros backend and an object of profile overrides')
    gap = config['gripper_max_gap_m']
    if type(gap) not in (int, float) or not math.isfinite(gap) or gap <= 0:
        raise ValueError('gripper_max_gap_m must be finite and positive')
    poses = config['named_poses']
    if type(poses) is not dict or set(poses) - {'left', 'right'}:
        raise ValueError('named_poses must map left/right to named pose dictionaries')
    for side, names in poses.items():
        if type(names) is not dict:
            raise ValueError('Named poses must be dictionaries')
        for name, pose in names.items():
            if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*', name):
                raise ValueError('Invalid pose name')
            if type(pose) is not dict or set(pose) != {'positions_rad', 'duration_sec'}:
                raise ValueError('Pose needs positions_rad and duration_sec')
            if (type(pose['positions_rad']) is not list or len(pose['positions_rad']) != 6 or
                    any(type(v) not in (int, float) or not math.isfinite(v) for v in pose['positions_rad'])):
                raise ValueError('A pose must contain six finite joint angles in radians')
            if type(pose['duration_sec']) not in (int, float) or not 0.1 <= pose['duration_sec'] <= 10:
                raise ValueError('Pose duration must be in [0.1,10] seconds')
    return config


class SkillSet:
    """Bounded invocation ledger and local motion owner (not a global ROS lease)."""
    def __init__(self, config, backend, *, cancel_timeout=3.0, cameras=None, grasp=None, arm=None):
        self._config, self.backend = deepcopy(config), backend
        self.cancel_timeout = cancel_timeout
        self.records = OrderedDict()
        self.blocked = False
        self.motion_owner = None
        self._background = set()
        side = {'type': 'string', 'enum': ['left', 'right']}
        self._definitions = {}
        self._handlers = {}

        self.register('get_robot_state', 'Read current readiness from ' + config['backend'] + ' backend.',
                      {}, {'ready': {'type': 'boolean'}, 'backend': {'type': 'string'}}, self._state)
        available = {key: value for key, value in config['named_poses'].items() if value}
        if available:
            common = set.intersection(*(set(value) for value in available.values()))
            if common:
                self.register('move_to_named_pose',
                              f'{config["backend"]}: reach a locally taught joint pose; no object-grasp claim.',
                              {'side': {'type': 'string', 'enum': sorted(available)},
                               'pose': {'type': 'string', 'enum': sorted(common)}},
                              {'side': {'type': 'string', 'enum': sorted(available)},
                               'pose': {'type': 'string', 'enum': sorted(common)}}, self._move, motion=True)
        self.register('set_gripper_gap',
                      f'{config["backend"]}: set total nominal gap in meters, configured current limit; '
                      'no Newton force control or proof of grasp.',
                      {'side': side, 'gap_m': {'type': 'number', 'minimum': 0,
                                             'maximum': config['gripper_max_gap_m'], 'unit': 'm'}},
                      {'side': side, 'position_m': {'type': 'number', 'unit': 'm'},
                       'reached_goal': {'type': 'boolean'}}, self._gripper, motion=True)
        if cameras is not None:
            cameras.register(self)
        if grasp is not None:
            grasp.register(self)
        if arm is not None:
            arm.register(self)
        self._manifest = {'schema': 'hei.skills/v1', 'backend': config['backend'], 'configuration_digest': digest(self._config),
                          'skills': [self._definitions[key] for key in sorted(self._definitions)]}
        self.revision = digest(self._manifest)
        self._manifest['revision'] = self.revision

    def register(self, name, description, arguments, outputs, handler, *, motion=False, timeout=None):
        if hasattr(self, '_manifest'):
            raise RuntimeError('Published skill sets are immutable; construct a new set to change capabilities')
        if name in self._definitions:
            raise ValueError(f'Duplicate skill: {name}')
        if set(arguments) & {'revision', 'invocation_id'}:
            raise ValueError('Reserved skill argument name')
        self._definitions[name] = {'name': name, 'description': description, 'arguments': deepcopy(arguments),
                                   'outputs': deepcopy(outputs), 'outcomes': ['success', 'failure'],
                                   'version': '1', 'motion': motion, 'timeout_sec': timeout if timeout is not None else (15.0 if motion else 2.0)}
        self._handlers[name] = handler

    def manifest(self):
        return deepcopy(self._manifest)

    @property
    def config(self):
        return deepcopy(self._config)

    async def _state(self, arguments, invocation):
        return {'ready': bool(self.backend.ready()) and not self.blocked, 'backend': self._config['backend']}

    async def _move(self, arguments, invocation):
        if not self.backend.ready():
            raise RuntimeError('Robot feedback/controllers not ready')
        pose = self._config['named_poses'][arguments['side']][arguments['pose']]
        await self.backend.move(invocation, arguments['side'], deepcopy(pose))
        return {'side': arguments['side'], 'pose': arguments['pose']}

    async def _gripper(self, arguments, invocation):
        if not self.backend.ready():
            raise RuntimeError('Robot feedback/controllers not ready')
        position = await self.backend.gripper(invocation, arguments['side'], arguments['gap_m'])
        return {'side': arguments['side'], 'position_m': position, 'reached_goal': True}

    def _remember(self, invocation, record):
        while len(self.records) >= 256:
            old = next((key for key, value in self.records.items() if value['task'] is None or value['task'].done()), None)
            if old is None:
                raise RuntimeError('Invocation ledger is full')
            del self.records[old]
        self.records[invocation] = record

    def status(self, invocation):
        record = self.records.get(invocation)
        if record is None:
            return {'status': 'unknown', 'cleanup_complete': False}
        return deepcopy(record['result'])

    async def invoke(self, name, arguments, *, invocation, revision):
        if revision != self.revision:
            raise ValueError('Skill catalog revision changed; rediscover and replan')
        if not isinstance(invocation, str) or not re.fullmatch(r'[a-zA-Z0-9:_-]{1,128}', invocation):
            raise ValueError('Invalid invocation ID')
        if invocation in self.records:
            raise ValueError('Invocation ID already used; no automatic replay')
        if name not in self._definitions:
            raise ValueError('Unknown skill')
        definition = self._definitions[name]
        fields(arguments, definition['arguments'])
        if definition['motion'] and (self.blocked or self.motion_owner is not None):
            raise RuntimeError('Motion busy or previous termination unconfirmed')
        record = {'task': None, 'started': False, 'result': {'status': 'running', 'cleanup_complete': False}}
        self._remember(invocation, record)
        if definition['motion']:
            self.motion_owner = invocation
        record['task'] = asyncio.create_task(self._perform(invocation, name, deepcopy(arguments), record))
        try:
            return await asyncio.shield(record['task'])
        except asyncio.CancelledError:
            await self.cancel(invocation)
            raise

    async def _perform(self, invocation, name, arguments, record):
        record['started'] = True
        definition = self._definitions[name]
        worker = asyncio.create_task(self._handlers[name](arguments, invocation))
        outcome, outputs, message, clean = 'failure', {}, '', True
        needs_cleanup = False
        try:
            done, _ = await asyncio.wait({worker}, timeout=definition['timeout_sec'])
            if not done:
                message, needs_cleanup = 'Skill execution timed out', True
            else:
                outputs = worker.result()
                fields(outputs, definition['outputs'])
                outcome = 'success'
        except asyncio.CancelledError:
            outcome, message, needs_cleanup = 'cancelled', 'Skill cancelled', True
        except Exception as exc:
            message, outputs, needs_cleanup = f'{type(exc).__name__}: {exc}', {}, True
        finally:
            if needs_cleanup:
                record['stopping'] = True
                worker.cancel()
                cleanup = asyncio.create_task(self.backend.cancel(invocation))
                done, pending = await asyncio.wait({worker, cleanup}, timeout=self.cancel_timeout)
                clean = not pending and cleanup in done and not cleanup.cancelled() and cleanup.exception() is None
                for task in done:
                    if not task.cancelled():
                        task.exception()
                for task in pending:
                    self._background.add(task)
                    task.add_done_callback(self._background_done)
                if not clean:
                    self.blocked = True
                    message += '; termination unconfirmed'
                outputs = {}
            if clean and self.motion_owner == invocation:
                self.motion_owner = None
            status = 'succeeded' if outcome == 'success' else 'cancelled' if outcome == 'cancelled' else 'failed'
            record['result'] = {'status': status, 'outcome': outcome, 'outputs': outputs, 'message': message,
                                'cleanup_complete': clean, 'revision': self.revision, 'invocation_id': invocation}
        return deepcopy(record['result'])

    def _background_done(self, task):
        self._background.discard(task)
        if not task.cancelled():
            task.exception()

    async def cancel(self, invocation):
        if not isinstance(invocation, str) or not re.fullmatch(r'[a-zA-Z0-9:_-]{1,128}', invocation):
            raise ValueError('Invalid invocation ID')
        record = self.records.get(invocation)
        if record is None:
            # Tombstone also covers cancellation racing with dispatch acceptance.
            self._remember(invocation, {'task': None, 'result': {'status': 'not_started', 'cleanup_complete': True}})
            return self.status(invocation)
        task = record['task']
        if task is not None and not task.done():
            if not record.get('started'):
                task.cancel()
                record['result'] = {'status': 'not_started', 'cleanup_complete': True}
                if self.motion_owner == invocation:
                    self.motion_owner = None
                await asyncio.gather(task, return_exceptions=True)
                return self.status(invocation)
            # Do not interrupt a cleanup already in progress a second time.
            if not record.get('cancel_requested') and not record.get('stopping'):
                record['cancel_requested'] = True
                task.cancel()
            done, _ = await asyncio.wait({task}, timeout=self.cancel_timeout + 0.5)
            if not done:
                self.blocked = True
        return self.status(invocation)

    async def stop(self):
        for invocation, record in list(self.records.items()):
            if record['task'] is not None and not record['task'].done():
                await self.cancel(invocation)


class MockSkillBackend:
    """Robot-side mock, deliberately distinct from the server-side demo connector."""
    def __init__(self, *, ready=True, delay=0.01):
        self.available, self.delay = ready, delay
        self.positions, self.gaps, self.calls = {}, {}, []

    def ready(self):
        return self.available

    async def move(self, invocation, side, pose):
        self.calls.append(invocation)
        await asyncio.sleep(self.delay)
        self.positions[side] = pose['positions_rad']

    async def gripper(self, invocation, side, gap):
        self.calls.append(invocation)
        await asyncio.sleep(self.delay)
        self.gaps[side] = gap
        return gap

    async def cancel(self, invocation):
        pass
