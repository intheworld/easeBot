"""Bounded latest-data cache; ROS callbacks never call the network SDK."""
from dataclasses import dataclass
import json
import math
import threading
import time

import cv2
import numpy as np

CAMERAS = ('front', 'left_wrist', 'right_wrist')


@dataclass(frozen=True)
class Frame:
    data: bytes
    width: int
    height: int
    stamp: tuple[int, int]
    received: float
    wall_time: float


class RobotCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.positions = {}
        self.joint_stamp = None
        self.last_joint = None
        self.joint_valid = False
        self.sequence = 0
        self.manager = {}
        self.mode = 'UNKNOWN'
        self.last_manager = None
        self.frames = {}
        self.base = None
        self.last_base = None

    def joints(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.lock:
            if self.joint_stamp == stamp:
                return
            self.joint_stamp = stamp
            self.joint_valid = (bool(msg.name) and len(msg.name) == len(msg.position)
                                and len(set(msg.name)) == len(msg.name)
                                and all(math.isfinite(v) for v in msg.position))
            if self.joint_valid:
                self.positions = dict(zip(msg.name, msg.position))
                self.last_joint = time.monotonic()
                self.sequence += 1

    def diagnostics(self, msg):
        for status in msg.status:
            if status.name == 'hei/robot_manager':
                with self.lock:
                    self.manager = {v.key: v.value for v in status.values}
                    self.mode = status.message
                    self.last_manager = time.monotonic()

    def odometry(self, msg):
        values = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        if all(math.isfinite(v) for v in values):
            with self.lock:
                self.base = values
                self.last_base = time.monotonic()

    def image(self, name, msg):
        if name not in CAMERAS or 'jpeg' not in msg.format.lower() or not 0 < len(msg.data) <= 2 * 1024 * 1024:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.lock:
            if name in self.frames and self.frames[name].stamp == stamp:
                return
        data = bytes(msg.data)
        if not data.startswith(b'\xff\xd8'):
            return
        # Validate actual JPEG dimensions; never fabricate camera calibration or FOV.
        image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[0] > 4096 or image.shape[1] > 4096:
            return
        frame = Frame(data, image.shape[1], image.shape[0], stamp, time.monotonic(), time.time())
        with self.lock:
            self.frames[name] = frame

    def frame(self, name):
        with self.lock:
            frame = self.frames.get(name)
            return frame if frame and time.monotonic() - frame.received < 1.0 else None

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            manager_live = self.last_manager is not None and now - self.last_manager < 2.0
            joint_live = self.last_joint is not None and now - self.last_joint < 1.0
            try:
                controllers = json.loads(self.manager.get('controllers', '{}'))
                if not isinstance(controllers, dict) or not all(isinstance(k, str) and isinstance(v, str) for k, v in controllers.items()):
                    controllers = {}
            except (ValueError, TypeError):
                controllers = {}
            return {
                'read_only': True, 'mode': self.mode if manager_live else 'UNKNOWN',
                'backend': self.manager.get('backend', 'unknown'),
                'enabled': manager_live and self.mode == 'ENABLED',
                'hardware_connected': self.manager.get('hardware_connected', 'unknown') if manager_live else 'unknown',
                'homed': self.manager.get('homed', 'unknown') if manager_live else 'unknown',
                'manager_online': manager_live, 'joint_valid': self.joint_valid and joint_live,
                'valid': self.joint_valid and joint_live and manager_live,
                'joint_age_s': now - self.last_joint if self.last_joint is not None else None,
                'sample_id': self.sequence, 'positions': dict(self.positions),
                'controllers': controllers if manager_live else {},
                'fault': self.manager.get('fault', '') if manager_live else 'manager diagnostics unavailable',
                'base_velocity': list(self.base) if self.last_base is not None and now - self.last_base < 1.0 else None,
                'cameras': {name: {'valid': name in self.frames and now - self.frames[name].received < 1.0,
                                  'age_s': now - self.frames[name].received if name in self.frames else None}
                            for name in CAMERAS},
            }
