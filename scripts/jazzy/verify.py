#!/usr/bin/env python3
"""After sourcing Jazzy, verify native C++ <-> Python DDS communication."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    import rclpy
    from std_msgs.msg import String

    prefix = Path.home() / 'ros2_jazzy/install'
    assert os.environ.get('ROS_DISTRO') == 'jazzy'
    assert str(prefix) in rclpy.__file__, rclpy.__file__
    assert not any('/opt/ros/humble' in entry for entry in sys.path), sys.path
    cli = subprocess.run(['ros2', '--help'], check=True, capture_output=True, text=True)
    assert 'Failed to load entry point' not in cli.stdout + cli.stderr, cli.stdout + cli.stderr
    rclpy.init(args=[])
    node = rclpy.create_node('jazzy_install_acceptance')
    samples = []
    node.create_subscription(String, '/jazzy_install/cpp', lambda msg: samples.append(msg.data), 10)
    publisher = node.create_publisher(String, '/jazzy_install/python', 10)
    processes = []
    try:
        with tempfile.TemporaryDirectory(prefix='jazzy-smoke-') as tmp:
            for executable, topic in [('talker', '/jazzy_install/cpp'), ('listener', '/jazzy_install/python')]:
                output = open(Path(tmp) / (executable + '.log'), 'w')
                try:
                    processes.append(subprocess.Popen(
                        [str(prefix / 'lib/demo_nodes_cpp' / executable), '--ros-args', '-r', 'chatter:=' + topic],
                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True))
                finally:
                    output.close()
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                assert all(process.poll() is None for process in processes), 'C++ demo exited unexpectedly'
                publisher.publish(String(data='hei-jazzy-python-to-cpp'))
                rclpy.spin_once(node, timeout_sec=.1)
                if samples and 'hei-jazzy-python-to-cpp' in (Path(tmp) / 'listener.log').read_text():
                    print('PASS: ROS_DISTRO=jazzy; rclpy=' + rclpy.__file__)
                    print('PASS: native C++ publisher -> Python subscriber: ' + samples[-1])
                    print('PASS: Python publisher -> native C++ subscriber')
                    return
            for name in ('talker', 'listener'):
                print((Path(tmp) / (name + '.log')).read_text())
            raise RuntimeError('Timed out verifying bidirectional DDS communication')
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
