#!/usr/bin/env python3
"""Reuse compatible ROS Rust build plugins, or install them inside .deps/."""
import argparse
from importlib import metadata
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
VERSIONS = {'colcon-cargo': '0.2.0', 'colcon-ros-cargo': '0.2.0',
            'cargo-ament-build': '0.1.11'}


def available():
    for name, expected in VERSIONS.items():
        try:
            if metadata.version(name) != expected:
                return False
        except metadata.PackageNotFoundError:
            return False
    return shutil.which('cargo-ament-build') is not None


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    if available():
        print('ROS Rust build plugins already available.')
        return
    requirement = ROOT / 'scripts/setup/build-requirements.txt'
    destination = ROOT / '.deps/build-python'
    if shutil.which('uv'):
        command = ['uv', 'pip', 'install', '--python', sys.executable]
    elif subprocess.run([sys.executable, '-m', 'pip', '--version'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
        command = [sys.executable, '-m', 'pip', 'install']
    else:
        raise SystemExit('Install uv or python3-pip to prepare workspace Python dependencies.')
    subprocess.run(command + ['--target', str(destination), '--upgrade',
                             '--requirement', str(requirement)], check=True)
    print('Build plugins installed in .deps/build-python.')


if __name__ == '__main__':
    main()
