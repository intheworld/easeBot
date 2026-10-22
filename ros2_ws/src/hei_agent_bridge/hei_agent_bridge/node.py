"""ROS subscriber node and Agnes MHS client in one read-only process."""
import asyncio
import logging
import signal
import threading
from urllib.parse import urlsplit

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, JointState

from .cache import CAMERAS, RobotCache


class AgentBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('agent_bridge', namespace='/hei', **kwargs)
        readonly = ParameterDescriptor(read_only=True)
        self.declare_parameter('hub_url', 'ws://127.0.0.1:4180', descriptor=readonly)
        self.declare_parameter('device_id', 'hei-01', descriptor=readonly)
        self.declare_parameter('report_hz', 5.0, descriptor=readonly)
        self.cache = RobotCache()
        self.create_subscription(JointState, 'joint_states', self.cache.joints, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.cache.diagnostics, 10)
        self.create_subscription(Odometry, 'base_controller/odom', self.cache.odometry, qos_profile_sensor_data)
        for name in CAMERAS:
            self.create_subscription(CompressedImage, f'cameras/{name}/image_raw/compressed',
                                     lambda msg, n=name: self.cache.image(n, msg), qos_profile_sensor_data)


async def run(node):
    try:
        from .device import ObserverDevice
    except ImportError as exc:
        raise RuntimeError('Missing Agnes SDK: run scripts/setup/prepare-agent-bridge.sh and source scripts/env/agent-bridge.bash') from exc
    url = node.get_parameter('hub_url').value
    parsed = urlsplit(url)
    if parsed.scheme not in ('ws', 'wss') or not parsed.hostname or parsed.path not in ('', '/') or parsed.query or parsed.fragment:
        raise ValueError('hub_url must be a ws:// or wss:// base origin, without /ws/mhs')
    hz = float(node.get_parameter('report_hz').value)
    if not 1 <= hz <= 20:
        raise ValueError('report_hz must be in [1,20]')
    observer = ObserverDevice(node.cache, node.get_parameter('device_id').value)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    loop = asyncio.get_running_loop()
    stopping = asyncio.Event()
    done = loop.create_future()
    for signum in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(signum, stopping.set)

    def spin():
        error = None
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
        except Exception as exc:
            error = exc
        finally:
            def finished():
                if not done.done():
                    done.set_result(error)
            loop.call_soon_threadsafe(finished)

    worker = threading.Thread(target=spin, name='agent-bridge-ros', daemon=True)
    worker.start()
    connection = asyncio.create_task(observer.device.run(url))
    pump = asyncio.create_task(observer.pump(hz))
    stop = asyncio.create_task(stopping.wait())
    node.get_logger().info(f'Read-only MHS observer {observer.device.id} connecting to {url}')
    try:
        finished, _ = await asyncio.wait([connection, pump, stop, done], return_when=asyncio.FIRST_COMPLETED)
        if done in finished and done.result() is not None:
            raise done.result()
        for task in (connection, pump):
            if task in finished:
                task.result()
                raise RuntimeError('MHS client ended (registration refused or device ID replaced)')
    finally:
        for task in (connection, pump, stop):
            task.cancel()
        await asyncio.gather(connection, pump, stop, return_exceptions=True)
        executor.shutdown(timeout_sec=2.0)
        worker.join(timeout=2.0)
        for signum in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(signum)
        if worker.is_alive():
            raise RuntimeError('ROS executor did not stop')


def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = AgentBridge()
        asyncio.run(run(node))
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
