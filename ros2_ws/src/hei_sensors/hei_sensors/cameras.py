import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image, CompressedImage

NAMES = ('front', 'left_wrist', 'right_wrist')


class Cameras(Node):
    def __init__(self):
        super().__init__('cameras')
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('fps', 10.0)
        self.backend = self.get_parameter('backend').value
        self.fps = float(self.get_parameter('fps').value)
        if self.backend not in ('mock', 'opencv') or not 1 <= self.fps <= 30:
            raise ValueError('backend must be mock/opencv; fps must be [1,30]')
        self.stopped = threading.Event()
        self.lock = threading.Lock()
        self.frames = {n: None for n in NAMES}
        self.last = {n: 0.0 for n in NAMES}
        self.errors = {n: 'waiting for frame' for n in NAMES}
        self.sequence = {n: 0 for n in NAMES}
        self.raw = {n: self.create_publisher(Image, f'cameras/{n}/image_raw', qos_profile_sensor_data) for n in NAMES}
        self.compressed = {n: self.create_publisher(CompressedImage, f'cameras/{n}/image_raw/compressed', qos_profile_sensor_data) for n in NAMES}
        for n in NAMES:
            self.declare_parameter(n + '_device', f'/dev/hei_{n}_camera')
        self.workers = []
        if self.backend == 'opencv':
            for n in NAMES:
                worker = threading.Thread(target=self.capture, args=(n,), daemon=True)
                worker.start()
                self.workers.append(worker)
        self.create_timer(1 / self.fps, self.publish)
        self.diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_timer(0.5, self.diagnostics)

    def capture(self, name):
        device = self.get_parameter(name + '_device').value
        if device.isdecimal():
            device = int(device)
        while not self.stopped.is_set():
            cap = cv2.VideoCapture(device)
            try:
                if not cap.isOpened():
                    raise RuntimeError('camera open failed')
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                while not self.stopped.is_set():
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('camera read failed')
                    captured_stamp = self.get_clock().now().to_msg()
                    with self.lock:
                        self.frames[name] = (cv2.resize(frame, (640, 480)), captured_stamp)
                        self.last[name] = time.monotonic()
                        self.errors[name] = ''
                    self.stopped.wait(1 / self.fps)
            except Exception as exc:
                with self.lock:
                    self.errors[name] = str(exc)
                self.stopped.wait(2)
            finally:
                cap.release()

    def publish(self):
        for index, name in enumerate(NAMES):
            if self.backend == 'mock':
                image = np.full((480, 640, 3), (32, 25, 18), dtype=np.uint8)
                for x in range(0, 640, 40):
                    cv2.line(image, (x, 0), (x, 480), (60, 50, 38))
                for y in range(0, 480, 40):
                    cv2.line(image, (0, y), (640, y), (60, 50, 38))
                x = 80 + self.sequence[name] % 180
                cv2.rectangle(image, (x, 240), (x + 90, 330), ((180, 200, 55), (70, 160, 220), (220, 140, 90))[index], -1)
                cv2.putText(image, name + ' / MOCK', (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (230, 230, 230), 1)
                cv2.putText(image, f'frame {self.sequence[name]} - no camera connected', (24, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
                stamp = self.get_clock().now().to_msg()
                with self.lock:
                    self.last[name], self.errors[name] = time.monotonic(), ''
            else:
                with self.lock:
                    item = self.frames[name]
                    age = time.monotonic() - self.last[name]
                    if item is None or age > 1 or self.errors[name]:
                        continue
                    image, stamp = item
                    self.frames[name] = None  # Never republish an old frame with a new stamp.
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                with self.lock:
                    self.errors[name] = 'JPEG encoding failed'
                continue
            raw = Image()
            raw.header.stamp = stamp
            raw.header.frame_id = name + '_camera_optical_frame'
            raw.height, raw.width = image.shape[:2]
            raw.encoding, raw.is_bigendian, raw.step = 'bgr8', 0, raw.width * 3
            raw.data = image.tobytes()
            # Avoid serializing full raw images unless a consumer actually subscribes.
            if self.raw[name].get_subscription_count():
                self.raw[name].publish(raw)
            compressed = CompressedImage()
            compressed.header = raw.header
            compressed.format = 'bgr8; jpeg compressed bgr8'
            compressed.data = jpeg.tobytes()
            self.compressed[name].publish(compressed)
            self.sequence[name] += 1

    def diagnostics(self):
        result = DiagnosticArray()
        result.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for name in NAMES:
                age = time.monotonic() - self.last[name]
                good = age < 1 and not self.errors[name]
                status = DiagnosticStatus(name='hei/camera/' + name, hardware_id=str(self.get_parameter(name + '_device').value),
                                          level=DiagnosticStatus.OK if good else DiagnosticStatus.WARN,
                                          message='online' if good else self.errors[name] or 'stale')
                status.values = [KeyValue(key='backend', value=self.backend), KeyValue(key='age_s', value=str(age))]
                result.status.append(status)
        self.diag.publish(result)

    def close(self):
        self.stopped.set()
        for worker in self.workers:
            worker.join(timeout=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = Cameras()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
