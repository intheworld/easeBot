#!/usr/bin/env python3
"""Prepare local Rust message discovery and optional test_msgs C typesupport.

Source ROS before running. Dependencies stay in .deps; no sudo is required.
"""
import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import tempfile
import urllib.request


ROOT = Path(__file__).resolve().parents[2]
TEST_LIBRARY = "libtest_msgs__rosidl_generator_c.so"


def prepare_index(repo, distro, prefixes):
    """Supply colcon-ros-cargo's missing index, preferring the first ROS prefix."""
    overlay = repo / ".deps/ros-rust-index" / distro
    packages = {}
    for prefix in prefixes:
        if prefix == overlay:
            continue
        for manifest in sorted((prefix / "share").glob("*/rust/Cargo.toml")):
            packages.setdefault(manifest.parent.parent.name, (prefix, manifest.parent))
    for name, (prefix, crate) in packages.items():
        # Native registrations need no compatibility marker. Remove stale ones.
        marker = overlay / "share/ament_index/resource_index/rust_packages" / name
        link = overlay / "share" / name / "rust"
        if link.is_symlink():
            link.unlink()
        native = prefix / "share/ament_index/resource_index/rust_packages" / name
        if native.exists():
            marker.unlink(missing_ok=True)
            continue
        if link.exists():
            raise RuntimeError(f"Refusing to replace a non-symlink Rust crate: {link}")
        marker.parent.mkdir(parents=True, exist_ok=True)
        link.parent.mkdir(parents=True, exist_ok=True)
        link.symlink_to(crate.resolve(), target_is_directory=True)
        marker.touch()
    index = overlay / "share/ament_index/resource_index/rust_packages"
    if index.exists():
        for marker in index.iterdir():
            if marker.name not in packages:
                link = overlay / "share" / marker.name / "rust"
                if link.is_symlink():
                    link.unlink()
                marker.unlink()
        (overlay / "local_setup.bash").write_text(
            "# Rust message index; loaded by scripts/env/ros-rust.bash.\n")


def prepare_local_prefix(prefix):
    """Make an extracted test_msgs package usable by colcon and ROS tools."""
    (prefix / "local_setup.bash").write_text('''\
_hei_local_prefix="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="${_hei_local_prefix}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_hei_local_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
unset _hei_local_prefix
''')


def download_test_msgs(repo, distro):
    """Verify the deb against the configured apt metadata before extraction."""
    package = f"ros-{distro}-test-msgs"
    metadata = subprocess.check_output(["apt-cache", "show", package], text=True)
    for stanza in metadata.split("\n\n"):
        fields = dict(line.split(": ", 1) for line in stanza.splitlines() if ": " in line)
        if "Filename" in fields and "SHA256" in fields:
            break
    else:
        raise RuntimeError(f"No archive and checksum in apt metadata for {package}")
    url = "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/" + fields["Filename"]
    with tempfile.TemporaryDirectory(prefix="hei-test-msgs-") as temp:
        archive = Path(temp) / "package.deb"
        with urllib.request.urlopen(url, timeout=120) as response:
            archive.write_bytes(response.read())
        if hashlib.sha256(archive.read_bytes()).hexdigest() != fields["SHA256"]:
            raise RuntimeError(f"Archive checksum mismatch for {package}")
        destination = repo / ".deps/ros"
        destination.mkdir(parents=True, exist_ok=True)
        subprocess.run(["dpkg-deb", "--extract", str(archive), str(destination)], check=True)


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    distro = os.environ.get("ROS_DISTRO", "")
    if not re.fullmatch(r"[a-z][a-z0-9_]*", distro):
        raise SystemExit("Source the ROS setup.bash first (ROS_DISTRO is missing or invalid).")
    prefixes = [Path(p) for p in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if p]
    prepare_index(ROOT, distro, prefixes)
    if any((p / "lib" / TEST_LIBRARY).is_file() for p in prefixes + [Path(f"/opt/ros/{distro}")]):
        print("Native test_msgs is available; Rust message index prepared.")
        return
    prefix = ROOT / f".deps/ros/opt/ros/{distro}"
    if not (prefix / "lib" / TEST_LIBRARY).is_file():
        download_test_msgs(ROOT, distro)
    prepare_local_prefix(prefix)
    print("Local test_msgs is ready; source scripts/env/ros-rust.bash.")


if __name__ == "__main__":
    main()
