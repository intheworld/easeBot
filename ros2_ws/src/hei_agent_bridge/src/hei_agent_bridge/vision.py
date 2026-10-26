"""Bounded RGB-D snapshots and camera-frame geometry; no motor/LLM access."""
from collections import OrderedDict
import asyncio
from dataclasses import dataclass
import base64
import hashlib
import io
import json
import math
import time
import uuid

import cv2
import numpy as np

MAX_ASSET = 2 * 1024 * 1024


@dataclass(frozen=True)
class Depth:
    values: np.ndarray
    stamp: tuple
    frame_id: str
    received: float


class PerceptionCache:
    def __init__(self, cache):
        self.cache = cache
        self.depth = None
        self.info = None

    def depth_image(self, msg):
        """Input topic must be depth aligned to the front color optical frame."""
        if msg.encoding not in ('16UC1', '32FC1') or not 0 < msg.width * msg.height <= 640 * 480:
            return
        dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('u2' if msg.encoding == '16UC1' else 'f4'))
        if msg.step < msg.width * dtype.itemsize or len(msg.data) != msg.step * msg.height:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.cache.lock:
            if self.depth is not None and self.depth.stamp == stamp:
                return
        values = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                            strides=(msg.step, dtype.itemsize)).astype(np.float32)
        if msg.encoding == '16UC1':
            values *= .001  # ROS 16UC1 depth convention: millimeters.
        values[~np.isfinite(values) | (values <= 0)] = np.nan
        values.setflags(write=False)
        with self.cache.lock:
            self.depth = Depth(values, stamp, msg.header.frame_id, time.monotonic())

    def camera_info(self, msg):
        values = [*msg.k, *msg.d]
        if (not 0 < msg.width * msg.height <= 640 * 480 or len(msg.k) != 9
                or not all(math.isfinite(v) for v in values) or msg.k[0] <= 0 or msg.k[4] <= 0):
            return
        info = {'width': msg.width, 'height': msg.height, 'frame_id': msg.header.frame_id,
                'k': list(msg.k), 'd': list(msg.d), 'distortion_model': msg.distortion_model}
        with self.cache.lock:
            self.info = info


class CameraSkills:
    """Snapshots are immutable, expire after 60s, and keep at most eight frames."""
    def __init__(self, cache, *, simulated=False):
        self.cache, self.simulated = cache, simulated
        self.snapshots = OrderedDict()

    def register(self, skills):
        skills.register('get_camera_observation',
            'Capture a fresh RGB snapshot from front/left_wrist/right_wrist. Front may include aligned '
            'metric depth and calibration. Returns asset references, not semantic object recognition.',
            {'camera': {'type': 'string', 'enum': ['front', 'left_wrist', 'right_wrist']}},
            {'observation_id': {'type': 'string'}, 'camera': {'type': 'string'},
             'width': {'type': 'number'}, 'height': {'type': 'number'},
             'rgb_ref': {'type': 'string'}, 'depth_ref': {'type': 'string'}, 'metadata_ref': {'type': 'string'},
             'depth_available': {'type': 'boolean'}, 'calibrated': {'type': 'boolean'},
             'frame_id': {'type': 'string'}, 'stamp_sec': {'type': 'number'},
             'simulated': {'type': 'boolean'}}, self.capture)
        skills.register('sample_depth_point',
            'On a captured front RGB-D observation, estimate camera-optical-frame XYZ from a normalized '
            'image point (u,v in [0,1], center=.5,.5). Uses median valid depth in a 5x5 neighborhood; '
            'z is axial depth, distance is Euclidean range. Requires aligned depth/calibration; no base-frame pose.',
            {'observation_id': {'type': 'string'}, 'u': {'type': 'number', 'minimum': 0, 'maximum': 1},
             'v': {'type': 'number', 'minimum': 0, 'maximum': 1}},
            {'x_m': {'type': 'number', 'unit': 'm'}, 'y_m': {'type': 'number', 'unit': 'm'},
             'z_m': {'type': 'number', 'unit': 'm'}, 'distance_m': {'type': 'number', 'unit': 'm'},
             'valid_samples': {'type': 'number'}, 'frame_id': {'type': 'string'},
             'simulated': {'type': 'boolean'}}, self.sample)

    def _get(self, observation_id):
        snapshot = self.snapshots.get(observation_id)
        if snapshot is None or time.monotonic() - snapshot['created'] > 60:
            self.snapshots.pop(observation_id, None)
            raise RuntimeError('Camera observation expired or not found; capture again')
        return snapshot

    async def capture(self, arguments, invocation):
        name = arguments['camera']
        simulated = self.simulated
        if self.simulated:
            image = np.full((480, 640, 3), 48, dtype=np.uint8)
            cv2.putText(image, name + ' MOCK', (25, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
            rgb = cv2.imencode('.jpg', image)[1].tobytes()
            stamp_sec, frame_id = time.time(), name + '_color_optical_frame'
            depth = np.full((480, 640), 1.0, dtype=np.float32) if name == 'front' else None
            info = {'width': 640, 'height': 480, 'frame_id': frame_id,
                    'k': [600., 0., 319.5, 0., 600., 239.5, 0., 0., 1.],
                    'd': [0.] * 5, 'distortion_model': 'plumb_bob'} if depth is not None else None
            width, height = 640, 480
        else:
            frame = self.cache.frame(name)
            if frame is None:
                raise RuntimeError(f'{name}: no fresh RGB image')
            rgb, width, height = frame.data, frame.width, frame.height
            simulated = frame.frame_id.startswith('mock_')
            stamp_sec, frame_id = frame.stamp[0] + frame.stamp[1] * 1e-9, frame.frame_id
            depth, info = None, None
            if name == 'front':
                # Compressed RGB and aligned depth arrive as separate ROS
                # callbacks. Allow the matching pair to finish arriving.
                for _ in range(5):
                    with self.cache.lock:
                        pending = self.cache.perception.depth
                        matched = pending is not None and pending.stamp == frame.stamp
                    if matched:
                        break
                    await asyncio.sleep(.02)
                with self.cache.lock:
                    sample, calibration = self.cache.perception.depth, self.cache.perception.info
                    if (sample is not None and sample.stamp == frame.stamp and sample.frame_id == frame_id
                            and frame_id and time.monotonic() - sample.received < 1
                            and sample.values.shape == (height, width)):
                        depth = sample.values
                        if (calibration is not None and calibration['frame_id'] == frame_id
                                and calibration['width'] == width and calibration['height'] == height):
                            info = calibration
        observation_id = uuid.uuid4().hex
        assets = {'rgb': ('image/jpeg', rgb)}
        if depth is not None:
            stream = io.BytesIO()
            np.save(stream, depth, allow_pickle=False)
            assets['depth'] = ('application/x-npy', stream.getvalue())
        metadata = {'camera': name, 'frame_id': frame_id, 'stamp_sec': stamp_sec,
                    'width': width, 'height': height, 'depth_unit': 'm',
                    'depth_aligned_to_rgb': depth is not None, 'calibration': info, 'simulated': simulated}
        assets['metadata'] = ('application/json', json.dumps(metadata, allow_nan=False).encode())
        if any(len(data) > MAX_ASSET for mime, data in assets.values()):
            raise RuntimeError('Camera asset too large; configure RGB-D at 640x480 or lower')
        for key in list(self.snapshots):
            if time.monotonic() - self.snapshots[key]['created'] > 60:
                self.snapshots.pop(key)
        while len(self.snapshots) >= 8:
            self.snapshots.popitem(last=False)
        self.snapshots[observation_id] = {'created': time.monotonic(), 'camera': name, 'assets': assets, 'depth': depth,
                                          'info': info, 'simulated': simulated}
        ref = f'mhs-observation:{observation_id}/'
        return {'observation_id': observation_id, 'camera': name, 'width': width, 'height': height,
                'rgb_ref': ref + 'rgb', 'depth_ref': ref + 'depth' if depth is not None else '',
                'metadata_ref': ref + 'metadata', 'depth_available': depth is not None,
                'calibrated': info is not None, 'frame_id': frame_id, 'stamp_sec': stamp_sec,
                'simulated': simulated}

    async def sample(self, arguments, invocation):
        snapshot = self._get(arguments['observation_id'])
        depth, info = snapshot['depth'], snapshot['info']
        if depth is None or info is None:
            raise RuntimeError('Observation has no aligned depth/calibration')
        height, width = depth.shape
        x, y = round(arguments['u'] * (width - 1)), round(arguments['v'] * (height - 1))
        patch = depth[max(0, y - 2):y + 3, max(0, x - 2):x + 3]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size < 3:
            raise RuntimeError('Not enough valid depth samples at this pixel')
        z = float(np.median(valid))
        k = np.array(info['k'], dtype=float).reshape(3, 3)
        if info['distortion_model'] not in ('plumb_bob', 'rational_polynomial', ''):
            raise RuntimeError('Unsupported distortion model for XYZ; use rectified calibration')
        ray = cv2.undistortPoints(np.array([[[x, y]]], dtype=float), k,
                                 np.array(info['d'], dtype=float) if info['d'] else None)[0, 0]
        xyz = [float(ray[0] * z), float(ray[1] * z), z]
        if not all(math.isfinite(v) for v in xyz):
            raise RuntimeError('Invalid camera projection')
        return {'x_m': xyz[0], 'y_m': xyz[1], 'z_m': z, 'distance_m': math.sqrt(sum(v*v for v in xyz)),
                'valid_samples': float(valid.size), 'frame_id': info['frame_id'], 'simulated': snapshot['simulated']}

    def read_asset(self, observation_id, kind):
        snapshot = self._get(observation_id)
        if kind not in snapshot['assets']:
            raise ValueError('Observation asset not available')
        mime, data = snapshot['assets'][kind]
        return {'mime_type': mime, 'byte_length': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
                'data_base64': base64.b64encode(data).decode('ascii')}
