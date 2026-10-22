from enum import Enum

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image, CompressedImage

NAMES = ('front', 'left_wrist', 'right_wrist')


class CameraPhase(Enum):
    UNCONFIGURED = 'unconfigured'
    INACTIVE = 'inactive'
    ACTIVE = 'active'
    FINALIZED = 'finalized'


class Cameras(LifecycleNode):
    """Three cameras share one lifecycle; diagnostics remain available while inactive."""

    WORKER_JOIN_TIMEOUT = 2.0

    def __init__(self, **kwargs):
        super().__init__('cameras', **kwargs)
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('capture_api', 'auto')
        for name in NAMES:
            self.declare_parameter(name + '_device', f'/dev/hei_{name}_camera')
            self.declare_parameter(name + '_fourcc', '')
        self.backend = 'unknown'
        self.fps = 10.0
        self.phase: CameraPhase = CameraPhase.UNCONFIGURED
        self.configured = False
        self.active = False
        self.stopped = threading.Event()
        self.stopped.set()
        self.lock = threading.Lock()
        self.frames = {n: None for n in NAMES}
        self.last = {n: 0.0 for n in NAMES}
        self.errors = {n: self.phase.value for n in NAMES}
        self.sequence = {n: 0 for n in NAMES}
        self.raw = {}
        self.compressed = {}
        self.captures = {}
        self.workers = []
        self.publish_timer = None
        self.add_on_set_parameters_callback(self.check_parameters)
        # An ordinary publisher intentionally keeps management diagnostics visible
        # before configure and after deactivate; only image output is lifecycle-gated.
        self.diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.diag_timer = self.create_timer(0.5, self.diagnostics)

    def check_parameters(self, parameters):
        configuration = {'backend', 'fps', 'capture_api'} | {
            n + suffix for n in NAMES for suffix in ('_device', '_fourcc')}
        if self.configured and any(p.name in configuration for p in parameters):
            return SetParametersResult(successful=False, reason='cleanup before changing camera configuration')
        return SetParametersResult(successful=True)

    def on_configure(self, state):
        try:
            self.backend = self.get_parameter('backend').value
            self.fps = float(self.get_parameter('fps').value)
            if self.backend not in ('mock', 'opencv') or not 1 <= self.fps <= 30:
                raise ValueError('backend must be mock/opencv; fps must be [1,30]')
            if self.backend == 'opencv':
                api = self.get_parameter('capture_api').value
                if api not in ('auto', 'v4l2'):
                    raise ValueError('capture_api must be auto/v4l2')
                for name in NAMES:
                    device = str(self.get_parameter(name + '_device').value)
                    device = int(device) if device.isdecimal() else device
                    cap = cv2.VideoCapture(device, cv2.CAP_V4L2) if api == 'v4l2' else cv2.VideoCapture(device)
                    self.captures[name] = cap
                    if not cap.isOpened():
                        raise RuntimeError(f'{name}: camera open failed ({device})')
                    fourcc = self.get_parameter(name + '_fourcc').value
                    if fourcc:
                        if not isinstance(fourcc, str) or len(fourcc) != 4 or not fourcc.isascii() or not fourcc.isalnum():
                            raise ValueError(f'{name}: fourcc must be empty or four ASCII letters/digits')
                        code = cv2.VideoWriter_fourcc(*fourcc)
                        if not cap.set(cv2.CAP_PROP_FOURCC, code) or int(cap.get(cv2.CAP_PROP_FOURCC)) != code:
                            raise RuntimeError(f'{name}: requested camera format {fourcc} is not supported')
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            for name in NAMES:
                self.raw[name] = self.create_lifecycle_publisher(
                    Image, f'cameras/{name}/image_raw', qos_profile_sensor_data)
                self.compressed[name] = self.create_lifecycle_publisher(
                    CompressedImage, f'cameras/{name}/image_raw/compressed', qos_profile_sensor_data)
            self.publish_timer = self.create_timer(1 / self.fps, self.publish)
            self.publish_timer.cancel()
            result = super().on_configure(state)
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError('managed entity configuration failed')
            self.configured = True
            self.phase = CameraPhase.INACTIVE
            self._clear_frames(self.phase.value)
            self.get_logger().info(f'Configured {self.backend} cameras; image output inactive')
            return result
        except Exception as exc:
            self.get_logger().error(f'Camera configure failed: {exc}')
            self._dispose()
            self.phase = CameraPhase.UNCONFIGURED
            self._clear_frames(str(exc))
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        try:
            if not self.configured or any(w.is_alive() for w in self.workers):
                raise RuntimeError('cameras are not configured or old capture workers remain')
            self._clear_frames('waiting for frame')
            result = super().on_activate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                return result
            self.stopped.clear()
            if self.backend == 'opencv':
                for name in NAMES:
                    worker = threading.Thread(target=self.capture, args=(name,), daemon=True)
                    worker.start()
                    self.workers.append(worker)
            self.active = True
            self.phase = CameraPhase.ACTIVE
            self.publish_timer.reset()
            self.get_logger().info('Camera acquisition and image publishing activated')
            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f'Camera activate failed: {exc}')
            return TransitionCallbackReturn.ERROR

    def on_deactivate(self, state):
        self.active = False
        if self.publish_timer is not None:
            self.publish_timer.cancel()
        result = super().on_deactivate(state)
        if not self._stop_workers():
            return TransitionCallbackReturn.ERROR
        self.phase = CameraPhase.INACTIVE
        self._clear_frames(self.phase.value)
        return result

    def on_cleanup(self, state):
        result = super().on_cleanup(state)
        if not self._dispose():
            return TransitionCallbackReturn.ERROR
        self.phase = CameraPhase.UNCONFIGURED
        self._clear_frames(self.phase.value)
        return result

    def on_shutdown(self, state):
        super().on_deactivate(state)
        if not self._dispose():
            return TransitionCallbackReturn.ERROR
        self.phase = CameraPhase.FINALIZED
        self._clear_frames(self.phase.value)
        return super().on_shutdown(state)

    def on_error(self, state):
        super().on_deactivate(state)
        if not self._dispose():
            # Do not release a VideoCapture still in use, or permit reconfiguration
            # that could start a second worker on the same device.
            self.phase = CameraPhase.FINALIZED
            return TransitionCallbackReturn.FAILURE
        self.phase = CameraPhase.UNCONFIGURED
        self._clear_frames('transition error; configure required')
        return super().on_error(state)

    def _clear_frames(self, error):
        with self.lock:
            for name in NAMES:
                self.frames[name] = None
                self.last[name] = 0.0
                self.errors[name] = error

    def _stop_workers(self):
        self.stopped.set()
        deadline = time.monotonic() + self.WORKER_JOIN_TIMEOUT
        for worker in self.workers:
            worker.join(timeout=max(0.0, deadline - time.monotonic()))
        self.workers = [w for w in self.workers if w.is_alive()]
        if self.workers:
            self.get_logger().error('Capture worker did not stop; refusing camera reuse')
            return False
        return True

    def _dispose(self):
        self.active = False
        if self.publish_timer is not None:
            self.publish_timer.cancel()
        if not self._stop_workers():
            return False
        for cap in self.captures.values():
            cap.release()
        self.captures.clear()
        if self.publish_timer is not None:
            self.destroy_timer(self.publish_timer)
            self.publish_timer = None
        for publisher in list(self.raw.values()) + list(self.compressed.values()):
            self.destroy_lifecycle_publisher(publisher)
        self.raw.clear()
        self.compressed.clear()
        self.configured = False
        return True

    def capture(self, name):
        cap = self.captures[name]
        try:
            while not self.stopped.is_set():
                ok, frame = cap.read()
                if self.stopped.is_set():
                    break
                if not ok:
                    raise RuntimeError('camera read failed; deactivate/cleanup/configure to reopen')
                image = cv2.resize(frame, (640, 480))
                captured_stamp = self.get_clock().now().to_msg()
                with self.lock:
                    self.frames[name] = (image, captured_stamp)
                    self.last[name] = time.monotonic()
                    self.errors[name] = ''
                self.stopped.wait(1 / self.fps)
        except Exception as exc:
            if not self.stopped.is_set():
                with self.lock:
                    self.errors[name] = str(exc)

    def publish(self):
        if not self.active:
            return
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
                good = self.active and age < 1 and not self.errors[name]
                status = DiagnosticStatus(name='hei/camera/' + name, hardware_id=str(self.get_parameter(name + '_device').value),
                                          level=DiagnosticStatus.OK if good else DiagnosticStatus.WARN,
                                          message=(self.phase.value if not self.active else 'online' if good else self.errors[name] or 'stale'))
                status.values = [KeyValue(key='backend', value=self.backend), KeyValue(key='age_s', value=str(age)),
                                 KeyValue(key='lifecycle_state', value=self.phase.value)]
                result.status.append(status)
        self.diag.publish(result)

    def close(self):
        # SIGINT can invalidate the ROS context before on_shutdown is called.
        # Resource cleanup must therefore also work without a lifecycle transition.
        return self._dispose()


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
