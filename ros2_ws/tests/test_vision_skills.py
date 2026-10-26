"""Synthetic RGB-D fixtures: no camera or motor access."""
import asyncio
from dataclasses import replace
import io
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.cache import RobotCache
from hei_agent_bridge.vision import CameraSkills


def cache_fixture(*, depth_stamp=1, depth_frame='front_optical', encoding='16UC1', invalid=False):
    cache = RobotCache()
    image = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', np.zeros((4, 4, 3), np.uint8))[1].tobytes())
    image.header.stamp.sec = 1
    image.header.frame_id = 'front_optical'
    cache.image('front', image)
    dtype, stride = ('>u2', 10) if encoding == '16UC1' else ('>f4', 20)
    raw = np.zeros((4, stride), np.uint8)
    values = np.ndarray((4, 4), dtype=dtype, buffer=raw, strides=(stride, np.dtype(dtype).itemsize))
    values[:] = (0 if invalid else 2000) if encoding == '16UC1' else (np.nan if invalid else 2.)
    depth = Image(width=4, height=4, encoding=encoding, is_bigendian=1, step=stride, data=raw.tobytes())
    depth.header.stamp.sec = depth_stamp
    depth.header.frame_id = depth_frame
    cache.perception.depth_image(depth)
    info = CameraInfo(width=4, height=4, k=[2., 0., 1., 0., 2., 1., 0., 0., 1.],
                      d=[0.] * 5, distortion_model='plumb_bob')
    info.header.frame_id = 'front_optical'
    cache.perception.camera_info(info)
    return cache, depth


@pytest.mark.parametrize('encoding', ['16UC1', '32FC1'])
def test_depth_units_stride_endianness_projection_and_assets(encoding):
    async def exercise():
        cache, _ = cache_fixture(encoding=encoding)
        skills = CameraSkills(cache)
        result = await skills.capture({'camera': 'front'}, 'test')
        assert result['depth_available'] and result['calibrated'] and not result['simulated']
        point = await skills.sample({'observation_id': result['observation_id'], 'u': 1/3, 'v': 1/3}, 'point')
        assert point['z_m'] == 2 and point['x_m'] == point['y_m'] == 0
        assert point['distance_m'] == 2 and point['frame_id'] == 'front_optical'
        depth_data = skills._get(result['observation_id'])['assets']['depth'][1]
        assert np.all(np.load(io.BytesIO(depth_data), allow_pickle=False) == 2.)
        meta = json.loads(skills._get(result['observation_id'])['assets']['metadata'][1])
        assert meta['depth_unit'] == 'm' and meta['depth_aligned_to_rgb']
    asyncio.run(exercise())


@pytest.mark.parametrize('changes', [{'depth_stamp': 2}, {'depth_frame': 'wrong_frame'}])
def test_unaligned_depth_is_not_used(changes):
    async def exercise():
        cache, _ = cache_fixture(**changes)
        skills = CameraSkills(cache)
        result = await skills.capture({'camera': 'front'}, 'test')
        assert not result['depth_available'] and not result['depth_ref']
        with pytest.raises(RuntimeError, match='no aligned depth'):
            await skills.sample({'observation_id': result['observation_id'], 'u': .5, 'v': .5}, 'point')
    asyncio.run(exercise())


def test_invalid_depth_stale_frames_and_duplicate_stamps():
    async def exercise():
        cache, depth = cache_fixture(invalid=True)
        skills = CameraSkills(cache)
        result = await skills.capture({'camera': 'front'}, 'test')
        with pytest.raises(RuntimeError, match='valid depth'):
            await skills.sample({'observation_id': result['observation_id'], 'u': .5, 'v': .5}, 'point')
        old = replace(cache.perception.depth, received=time.monotonic() - 2)
        cache.perception.depth = old
        cache.perception.depth_image(depth)
        assert cache.perception.depth is old
        cache.frames['front'] = replace(cache.frames['front'], received=time.monotonic() - 2)
        with pytest.raises(RuntimeError, match='fresh RGB'):
            await skills.capture({'camera': 'front'}, 'stale')
    asyncio.run(exercise())


def test_snapshot_bounds_expiry_and_mock_marking():
    async def exercise():
        skills = CameraSkills(RobotCache(), simulated=True)
        first = await skills.capture({'camera': 'front'}, 'first')
        assert first['simulated'] and first['depth_available']
        for i in range(8):
            last = await skills.capture({'camera': 'right_wrist'}, str(i))
        assert len(skills.snapshots) == 8 and not last['depth_available']
        with pytest.raises(RuntimeError, match='expired'):
            skills._get(first['observation_id'])
        skills._get(last['observation_id'])['created'] -= 61
        with pytest.raises(RuntimeError, match='expired'):
            skills.read_asset(last['observation_id'], 'rgb')
    asyncio.run(exercise())
