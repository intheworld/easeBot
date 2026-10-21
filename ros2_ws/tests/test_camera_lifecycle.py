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
