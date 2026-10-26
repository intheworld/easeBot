"""Lifecycle and camera resource ownership tests; never access physical devices."""
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.lifecycle import TransitionCallbackReturn as Return
from rclpy.parameter import Parameter
from hei_sensors.cameras import CameraPhase, Cameras, NAMES


@pytest.fixture
def camera():
    context = Context()
    rclpy.init(context=context)
    node = Cameras(context=context)
    try:
        yield node
    finally:
        node.close()
        node.destroy_node()
        context.shutdown()


class FakeCapture:
    def __init__(self, opened=True):
        self.opened = opened
        self.released = False
        self.reads = 0

    def isOpened(self):
        return self.opened

    def set(self, *_):
        return True

    def read(self):
        self.reads += 1
        return True, np.zeros((480, 640, 3), dtype=np.uint8)

    def release(self):
        self.released = True


def test_explicit_capture_format_and_rollback(camera, monkeypatch):
    import cv2
    opened, accepted = [], [True]

    class FormatCapture(FakeCapture):
        def set(self, prop, value):
            if prop == cv2.CAP_PROP_FOURCC:
                self.code = value
                return accepted[0]
            return True

        def get(self, prop):
            return self.code

    def open_capture(device, api):
        assert api == cv2.CAP_V4L2
        capture = FormatCapture(opened=True)
        opened.append(capture)
        return capture

    monkeypatch.setattr('hei_sensors.cameras.cv2.VideoCapture', open_capture)
    camera.set_parameters([Parameter('backend', value='opencv'), Parameter('capture_api', value='v4l2'),
                           Parameter('left_wrist_fourcc', value='MJPG')])
    assert camera.trigger_configure() == Return.SUCCESS
    assert opened[1].code == cv2.VideoWriter_fourcc(*'MJPG')
    assert not camera.set_parameters([Parameter('left_wrist_fourcc', value='YUYV')])[0].successful
    assert camera.trigger_cleanup() == Return.SUCCESS
    assert all(cap.released for cap in opened)
    accepted[0] = False
    opened.clear()
    assert camera.trigger_configure() == Return.FAILURE
    assert all(cap.released for cap in opened)
    assert not camera.captures


def test_capture_starts_only_when_active_and_cleanup_releases_devices(camera, monkeypatch):
    captures = []
    def open_capture(_):
        capture = FakeCapture()
        captures.append(capture)
        return capture
    monkeypatch.setattr('hei_sensors.cameras.cv2.VideoCapture', open_capture)
    camera.set_parameters([Parameter('backend', value='opencv')])
    assert camera.trigger_configure() == Return.SUCCESS
    assert len(captures) == len(NAMES)
    assert not any(c.reads for c in captures)
    assert not camera.set_parameters([Parameter('fps', value=15.0)])[0].successful
    assert camera.trigger_activate() == Return.SUCCESS
    deadline = time.monotonic() + 2
    while not all(c.reads for c in captures):
        assert time.monotonic() < deadline
        time.sleep(0.01)
    assert camera.trigger_deactivate() == Return.SUCCESS
    assert not camera.workers
    counts = [c.reads for c in captures]
    time.sleep(0.15)
    assert [c.reads for c in captures] == counts
    assert not any(c.released for c in captures)  # inactive retains configured devices
    assert camera.trigger_cleanup() == Return.SUCCESS
    assert all(c.released for c in captures)
    assert camera.set_parameters([Parameter('backend', value='mock')])[0].successful
    assert camera.trigger_configure() == Return.SUCCESS
    assert camera.trigger_activate() == Return.SUCCESS
    assert camera.trigger_shutdown() == Return.SUCCESS
    assert camera.phase is CameraPhase.FINALIZED
    assert not camera.raw and not camera.compressed


def test_configure_failure_rolls_back_and_can_be_retried(camera, monkeypatch):
    captures = []
    def open_capture(_):
        capture = FakeCapture(opened=len(captures) != 2)
        captures.append(capture)
        return capture
    monkeypatch.setattr('hei_sensors.cameras.cv2.VideoCapture', open_capture)
    camera.set_parameters([Parameter('backend', value='opencv')])
    assert camera.trigger_configure() == Return.FAILURE
    assert camera.phase is CameraPhase.UNCONFIGURED
    assert all(c.released for c in captures)
    assert not camera.captures and not camera.raw and camera.publish_timer is None
    camera.set_parameters([Parameter('backend', value='mock'), Parameter('fps', value=0.0)])
    assert camera.trigger_configure() == Return.FAILURE
    camera.set_parameters([Parameter('fps', value=10.0)])
    assert camera.trigger_configure() == Return.SUCCESS
    assert camera.trigger_cleanup() == Return.SUCCESS


def test_rgbd_lifecycle_uses_single_front_owner_and_publishes_metric_pair(camera, monkeypatch):
    from sensor_msgs.msg import Image
    import hei_sensors.realsense
    captured, depths, infos = [], [], []

    class RGBDCapture(FakeCapture):
        started = False
        def start(self):
            self.started = True
        def stop(self):
            self.started = False
        def read_rgbd(self):
            assert self.started
            self.reads += 1
            return np.zeros((480, 640, 3), np.uint8), np.full((480, 640), 2., np.float32), {
                'k': [500., 0., 319.5, 0., 500., 239.5, 0., 0., 1.],
                'd': [0.] * 5, 'distortion_model': 'plumb_bob'}

    front = RGBDCapture()
    monkeypatch.setattr(hei_sensors.realsense, 'RealSenseCapture', lambda serial: front)
    def open_wrist(device):
        assert 'front' not in str(device)
        capture = FakeCapture()
        captured.append(capture)
        return capture
    monkeypatch.setattr('hei_sensors.cameras.cv2.VideoCapture', open_wrist)
    camera.set_parameters([Parameter('backend', value='opencv'), Parameter('front_backend', value='realsense')])
    assert camera.trigger_configure() == Return.SUCCESS
    assert len(captured) == 2 and not front.started
    monkeypatch.setattr(camera.depth_pub, 'publish', depths.append)
    monkeypatch.setattr(camera.info_pub, 'publish', infos.append)
    assert camera.trigger_activate() == Return.SUCCESS
    deadline = time.monotonic() + 2
    while camera.frames['front'] is None:
        assert time.monotonic() < deadline
        time.sleep(.01)
    camera.publish()
    assert depths and infos and isinstance(depths[0], Image)
    assert depths[0].header == infos[0].header
    assert depths[0].encoding == '32FC1'
    assert np.all(np.frombuffer(depths[0].data, dtype='<f4') == 2.)
    assert camera.trigger_deactivate() == Return.SUCCESS and not front.started
    assert camera.trigger_cleanup() == Return.SUCCESS and front.released


def test_rgbd_reader_discards_startup_timestamp_mismatch():
    from types import SimpleNamespace as NS
    from hei_sensors.realsense import RealSenseCapture
    intrinsics = NS(fx=500., fy=500., ppx=1., ppy=1., coeffs=[0.] * 5, model='none')

    class Frame:
        profile = NS(as_video_stream_profile=lambda: NS(get_intrinsics=lambda: intrinsics))
        def __init__(self, timestamp, depth=False):
            self.timestamp, self.depth = timestamp, depth
        def get_timestamp(self):
            return self.timestamp
        def get_frame_timestamp_domain(self):
            return 'global'
        def get_data(self):
            return np.full((2, 3), 2000, np.uint16) if self.depth else np.zeros((2, 3, 3), np.uint8)

    pairs = iter([NS(get_color_frame=lambda: Frame(0.), get_depth_frame=lambda: Frame(133., True)),
                  NS(get_color_frame=lambda: Frame(200.), get_depth_frame=lambda: Frame(200.1, True))])
    reader = object.__new__(RealSenseCapture)
    reader.stop_requested = threading.Event()
    reader.pipeline = NS(wait_for_frames=lambda timeout: next(pairs))
    reader.align, reader.scale = NS(process=lambda frames: frames), .001
    reader.rs = NS(distortion=NS(none='none', brown_conrady='brown'))
    image, depth, info = reader.read_rgbd()
    assert image.shape == (2, 3, 3) and np.all(depth == 2.)
    assert info['k'][0] == 500.


def test_blocked_reader_is_not_released_or_reused(camera, monkeypatch):
    unblock = threading.Event()
    started = [threading.Event() for _ in NAMES]
    captures = []
    class BlockedCapture(FakeCapture):
        def read(self):
            started[captures.index(self)].set()
            unblock.wait(5)
            return super().read()
    def open_capture(_):
        capture = BlockedCapture()
        captures.append(capture)
        return capture
    monkeypatch.setattr('hei_sensors.cameras.cv2.VideoCapture', open_capture)
    camera.WORKER_JOIN_TIMEOUT = 0.02
    try:
        camera.set_parameters([Parameter('backend', value='opencv')])
        assert camera.trigger_configure() == Return.SUCCESS
        assert camera.trigger_activate() == Return.SUCCESS
        assert all(event.wait(2) for event in started)
        assert camera.trigger_deactivate() == Return.ERROR
        assert camera.phase is CameraPhase.FINALIZED
        assert not camera.active
        assert not any(c.released for c in captures)
        assert camera.workers
    finally:
        unblock.set()
        camera.WORKER_JOIN_TIMEOUT = 2
        assert camera.close()
        assert all(c.released for c in captures)
