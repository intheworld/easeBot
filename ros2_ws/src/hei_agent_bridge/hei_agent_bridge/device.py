"""MHS SDK is touched only from the asyncio thread."""
import asyncio

from agnes_mhs import Device, errors

from .cache import CAMERAS


class ObserverDevice:
    def __init__(self, cache, device_id):
        self.cache = cache
        # This represents a logical read-only observer, not a motion-capable robot.
        self.device = Device(device_id, 'sensor', name='HEI ReBot Lift read-only observer',
                             model='HEI ROS 2 observer', mobile=False, localization='none',
                             state={
                                 'mode': {'type': 'string'}, 'backend': {'type': 'string'},
                                 'valid': {'type': 'boolean'}, 'enabled': {'type': 'boolean'},
                                 'read_only': {'type': 'boolean'}, 'sample_id': {'type': 'integer'},
                                 'stop_scope': {'type': 'string'},
                             })
        self.telemetry = self.device.source('telemetry', 'values', 'ROS feedback availability', hz=2,
                                            fields={'valid': {'type': 'boolean'},
                                                    'sample_id': {'type': 'integer'}})
        self.images = {name: self.device.source(name, 'image', f'{name} ROS camera (uncalibrated)',
                                                mime='image/jpeg', encoding='rgb', hz=1)
                       for name in CAMERAS}
        self.last_frames = {}
        self.device.on_stop = self.stop

        @self.device.tool('Read the latest ROS state, ages and camera availability; does not move the robot.',
                          name='get_robot_state', timeout=5, read_only=True)
        async def get_robot_state(call):
            return {'detail': 'Read-only ROS snapshot; inspect valid and ages before using values.',
                    'data': self.cache.snapshot()}

        self.refresh()
        problems = errors('RegisterParams', self.device.description())
        if problems:
            raise ValueError('Invalid MHS device registration: ' + '; '.join(problems))

    def stop(self):
        # No motion is owned by this observer. In particular, do not claim to stop
        # controllers commanded independently by keyboard/ROS action clients.
        pass

    def refresh(self):
        snapshot = self.cache.snapshot()
        self.device.update(mode=snapshot['mode'], backend=snapshot['backend'], valid=snapshot['valid'],
                           enabled=snapshot['enabled'], read_only=True, sample_id=snapshot['sample_id'],
                           stop_scope='observer_only_no_robot_motion_control')
        self.device.problem = snapshot['fault'] or (None if snapshot['valid'] else 'ROS feedback missing or stale')
        if self.telemetry.wants():
            self.telemetry.send({'valid': snapshot['valid'], 'sample_id': snapshot['sample_id']})
        for name, source in self.images.items():
            if not source.wants():
                continue
            frame = self.cache.frame(name)
            if frame is not None and self.last_frames.get(name) is not frame:
                if source.send({'w': frame.width, 'h': frame.height}, frame.data, t=frame.wall_time):
                    self.last_frames[name] = frame

    async def pump(self, report_hz):
        while True:
            self.refresh()
            await asyncio.sleep(1.0 / report_hz)
