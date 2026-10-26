"""Publish the robot-owned skill catalog and guarded calls through MHS."""
from agnes_mhs import CallError, errors

from .discovery import DiscoveryDevice as Device

from .device import ObserverDevice
from .skills import MockSkillBackend, SkillSet
from .vision import CameraSkills


class SkillDevice(ObserverDevice):
    def __init__(self, cache, device_id, config, node):
        if config['backend'] == 'mock':
            backend = MockSkillBackend()
        else:
            from .ros_skills import RosSkillBackend
            backend = RosSkillBackend(node, config)
        self.cameras = CameraSkills(cache, simulated=config['backend'] == 'mock')
        self.grasp = None
        if 'right_grasp' in config:
            from .grasp.service import GraspSkills, prepare_config
            self.grasp = GraspSkills(prepare_config(config['right_grasp']), self.cameras, backend)
        self.arm_motion = None
        if 'arm_motion' in config or self.grasp is not None:
            from .arm.service import ArmMotionSkills, prepare_config as prepare_arm_config
            inherited = ({key:self.grasp.profile[key] for key in ('worker_python','joint_speed_rad_s','scene')}
                         if self.grasp is not None else {})
            self.arm_motion = ArmMotionSkills(prepare_arm_config({**inherited, **config.get('arm_motion',{})}), backend)
        self.skills = SkillSet(config, backend, cameras=self.cameras, grasp=self.grasp, arm=self.arm_motion)
        device = Device(device_id, 'robot', name='HEI skill set', model='HEI ROS 2 ' + config['backend'],
                        mobile=False, localization='none', resources={'manipulation': 'reject'},
                        state={key: {'type': kind} for key, kind in {
                            'mode': 'string', 'backend': 'string', 'valid': 'boolean',
                            'enabled': 'boolean', 'read_only': 'boolean', 'sample_id': 'integer',
                            'stop_scope': 'string', 'skill_revision': 'string', 'skills_blocked': 'boolean',
                        }.items()})
        super().__init__(cache, device_id, device=device)
        device.on_stop = self.stop

        @device.tool('Read robot-owned skill manifest and its revision.', timeout=3,
                     name='get_skill_catalog', read_only=True)
        async def catalog(call):
            return {'data': self.skills.manifest()}

        @device.tool('Read a bounded immutable observation asset; no file paths or arbitrary URLs.',
                     name='read_observation_asset', timeout=3, read_only=True,
                     params={'observation_id': {'type': 'string', 'maxLength': 32},
                             'kind': {'type': 'string', 'enum': ['rgb', 'depth', 'metadata']}},
                     required=['observation_id', 'kind'])
        async def read_asset(call, observation_id, kind):
            if call.notes:
                raise CallError('invalid', 'Changed asset arguments are not accepted')
            try:
                return {'data': self.cameras.read_asset(observation_id, kind)}
            except (ValueError, RuntimeError) as exc:
                raise CallError('invalid', str(exc)) from exc

        parameters = {'invocation_id': {'type': 'string', 'maxLength': 128}}

        @device.tool('Read one local execution, including termination confirmation.', timeout=3,
                     name='get_skill_execution', params=parameters, required=list(parameters), read_only=True)
        async def status(call, invocation_id):
            return {'data': self.skills.status(invocation_id)}

        @device.tool('Cancel only this skill invocation and confirm local termination.', timeout=5,
                     name='cancel_skill', params=parameters, required=list(parameters))
        async def cancel(call, invocation_id):
            if call.notes:
                raise CallError('invalid', 'Changed cancellation ID is not accepted')
            return {'data': await self.skills.cancel(invocation_id)}

        for definition in self.skills.manifest()['skills']:
            self._register(definition)
        self.refresh()
        problems = errors('RegisterParams', device.description())
        if problems:
            raise ValueError('Invalid skill device registration: ' + '; '.join(problems))

    def _register(self, definition):
        params = {**{key: {field: value for field, value in spec.items() if field != 'unit'}
                     for key, spec in definition['arguments'].items()}, 'revision': {'type': 'string'},
                  'invocation_id': {'type': 'string', 'maxLength': 128}}

        async def invoke(call, revision, invocation_id, **arguments):
            if call.notes:
                raise CallError('invalid', 'Clamped, dropped or defaulted skill arguments are not accepted')
            try:
                value = await self.skills.invoke(definition['name'], arguments,
                                                 invocation=invocation_id, revision=revision)
                return {'detail': value['message'], 'data': value}
            except (ValueError, RuntimeError) as exc:
                raise CallError('invalid', str(exc)) from exc

        self.device.tool(definition['description'], name=definition['name'], params=params,
                         required=list(params), timeout=definition['timeout_sec'] + 5,
                         motion=definition['motion'], read_only=not definition['motion'],
                         uses=['manipulation'] if definition['motion'] else [])(invoke)

    def snapshot(self):
        snapshot = self.cache.snapshot()
        ready = self.skills.backend.ready() and not self.skills.blocked
        snapshot.update(backend=self.skills.config['backend'], read_only=self.skills.config['backend'] == 'mock',
                        valid=ready, stop_scope='skill_owned_actions_only',
                        mode='SKILLS_BLOCKED' if self.skills.blocked else 'SKILLS_READY' if ready else 'SKILLS_WAITING',
                        fault='Skill termination unconfirmed; motion blocked' if self.skills.blocked else
                        '' if ready else 'Skill action servers or fresh feedback unavailable')
        return snapshot

    def refresh(self):
        super().refresh()
        self.device.update(skill_revision=self.skills.revision, skills_blocked=self.skills.blocked)

    async def stop(self):
        await self.skills.stop()

    def close(self):
        close = getattr(self.skills.backend, 'close', None)
        if close is not None:
            close()
