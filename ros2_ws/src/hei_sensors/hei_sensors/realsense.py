"""Single-owner D435 RGB-D capture. Importing this module never opens hardware."""
import numpy as np
import time
import threading
import math


class RealSenseCapture:
    def __init__(self, serial='', fps=15):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError('RealSense SDK missing; run scripts/setup.bash --camera-sdk') from exc
        self.rs, self.fps = rs, fps
        devices = [device for device in rs.context().query_devices()
                   if 'D435' in device.get_info(rs.camera_info.name)
                   and (not serial or device.get_info(rs.camera_info.serial_number) == serial)]
        if len(devices) != 1:
            raise RuntimeError('Expected one D435; check USB or set front_serial')
        self.serial = devices[0].get_info(rs.camera_info.serial_number)
        self.pipeline = None
        self.stop_requested = threading.Event()

    def isOpened(self):
        return True  # Device identified; streaming starts only on activation.

    def start(self):
        if self.pipeline is not None:
            return
        self.stop_requested.clear()
        rs = self.rs
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, self.fps)
        pipeline = rs.pipeline()
        try:
            profile = pipeline.start(config)
            self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
            if not math.isfinite(self.scale) or self.scale <= 0:
                raise RuntimeError('Invalid RealSense depth scale')
            self.align = rs.align(rs.stream.color)
            self.pipeline = pipeline
        except BaseException:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            raise

    def read_rgbd(self):
        deadline = time.monotonic() + 5.
        while True:
            if self.stop_requested.is_set():
                raise RuntimeError('RealSense capture stopped')
            try:
                frames = self.align.process(self.pipeline.wait_for_frames(500))
            except RuntimeError as exc:
                if "Frame didn't arrive" not in str(exc) or time.monotonic() >= deadline:
                    raise
                continue
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            # Startup sync can contain unmatched frames. Never compare clocks
            # from different domains or publish a bad pair as synchronized.
            if (color and depth and color.get_frame_timestamp_domain() == depth.get_frame_timestamp_domain()
                    and abs(color.get_timestamp() - depth.get_timestamp()) <= 50):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('No timestamp-compatible RGB-D pair within five seconds')
        image = np.asanyarray(color.get_data()).copy()
        meters = np.asanyarray(depth.get_data()).astype(np.float32) * self.scale
        intrinsics = color.profile.as_video_stream_profile().get_intrinsics()
        model = 'plumb_bob'
        if any(intrinsics.coeffs) and intrinsics.model not in (self.rs.distortion.none, self.rs.distortion.brown_conrady):
            # Preserve the actual model; geometry will reject unsupported XYZ
            # rather than treating inverse Brown coefficients as forward Brown.
            model = 'realsense_' + str(intrinsics.model).split('.')[-1]
        calibration = {'k': [intrinsics.fx, 0., intrinsics.ppx, 0., intrinsics.fy, intrinsics.ppy, 0., 0., 1.],
                       'd': list(intrinsics.coeffs), 'distortion_model': model}
        return image, meters, calibration

    def stop(self):
        self.request_stop()
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None

    def request_stop(self):
        self.stop_requested.set()

    def release(self):
        self.stop()
