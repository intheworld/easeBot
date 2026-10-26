#!/usr/bin/env python3
"""Install the pinned D435 SDK inside the workspace, never open the camera."""
import argparse
from importlib import metadata
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
VERSION = '2.58.4.10922'


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    destination = ROOT / '.deps/sensors-python'
    if any(dist.metadata['Name'] == 'pyrealsense2' and dist.version == VERSION
           for dist in metadata.distributions(path=[str(destination)])):
        print('Pinned RealSense SDK already prepared.')
        return
    command = (['uv', 'pip', 'install', '--python', sys.executable] if shutil.which('uv') else
               [sys.executable, '-m', 'pip', 'install'])
    subprocess.run(command + ['--target', str(destination), '--upgrade',
                             'pyrealsense2==' + VERSION], check=True)
    print('RealSense SDK prepared; source scripts/env/workspace.bash before launch.')


if __name__ == '__main__':
    main()
