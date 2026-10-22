"""Check local dependency indexing and archive caching without ROS or network."""
import hashlib
import importlib.util
import io
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[2]


def load_script(name, relative):
    spec = importlib.util.spec_from_file_location(name, REPO / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rust = load_script("prepare_rust_ros", "scripts/setup/prepare-rust-ros.py")
archives = load_script("jazzy_archives", "scripts/jazzy/_archives.py")


def test_rust_index_precedence_native_registration_and_cleanup(tmp_path):
    repo = tmp_path / "repo"
    high, low = tmp_path / "overlay", tmp_path / "underlay"
    for prefix in (high, low):
        crate = prefix / "share/sensor_msgs/rust"
        crate.mkdir(parents=True)
        (crate / "Cargo.toml").write_text('[package]\nname = "sensor_msgs"\n')
    index = repo / ".deps/ros-rust-index/jazzy"
    link = index / "share/sensor_msgs/rust"
    marker = index / "share/ament_index/resource_index/rust_packages/sensor_msgs"
    rust.prepare_index(repo, "jazzy", [high, low])
    assert link.resolve() == high / "share/sensor_msgs/rust"
    rust.prepare_index(repo, "jazzy", [index, high, low])
    assert link.resolve() == high / "share/sensor_msgs/rust"
    rust.prepare_index(repo, "jazzy", [low, high])
    assert link.resolve() == low / "share/sensor_msgs/rust"
    native = high / "share/ament_index/resource_index/rust_packages/sensor_msgs"
    native.parent.mkdir(parents=True)
    native.touch()
    rust.prepare_index(repo, "jazzy", [high, low])
    assert not marker.exists() and not link.is_symlink()
    rust.prepare_index(repo, "jazzy", [low])
    assert marker.exists()
    rust.prepare_index(repo, "jazzy", [])
    assert not marker.exists() and not link.is_symlink()


def test_rust_index_does_not_replace_local_source_directory(tmp_path):
    crate = tmp_path / "ros/share/sensor_msgs/rust"
    crate.mkdir(parents=True)
    (crate / "Cargo.toml").touch()
    local = tmp_path / ".deps/ros-rust-index/jazzy/share/sensor_msgs/rust"
    local.mkdir(parents=True)
    (local / "keep").touch()
    with pytest.raises(RuntimeError, match="non-symlink"):
        rust.prepare_index(tmp_path, "jazzy", [tmp_path / "ros"])
    assert (local / "keep").exists()


def test_archive_download_is_atomic_and_cleans_interrupted_data(tmp_path, monkeypatch):
    target = tmp_path / "source.tar.gz"
    monkeypatch.setattr(archives.urllib.request, "urlopen", lambda *args, **kwargs: io.BytesIO(b"complete"))
    archives.download("https://example.invalid/source", target)
    assert archives.sha256(target) == hashlib.sha256(b"complete").hexdigest()

    class Interrupted(io.BytesIO):
        def read(self, *_):
            raise OSError("connection interrupted")

    monkeypatch.setattr(archives.urllib.request, "urlopen", lambda *args, **kwargs: Interrupted())
    with pytest.raises(OSError, match="interrupted"):
        archives.download("https://example.invalid/source", target)
    assert target.read_bytes() == b"complete"
    assert not list(tmp_path.glob("*.part"))
