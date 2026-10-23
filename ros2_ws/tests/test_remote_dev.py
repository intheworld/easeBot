"""Exercise SSH orchestration locally with real rsync, without network access."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("remote_dev", REPO / "scripts/remote-dev.py")
remote_dev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote_dev)


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    if not shutil.which("rsync"):
        pytest.skip("rsync required")
    source = tmp_path / "source"
    destination = tmp_path / "remote" / "workspace"
    (source / "deploy").mkdir(parents=True)
    shutil.copy(REPO / "deploy/rsync.exclude", source / "deploy/rsync.exclude")
    (source / "hello.py").write_text("version = 1\n")
    config = tmp_path / "targets.json"
    config.write_text(json.dumps({"robot": {"host": "local-test", "path": str(destination)}}))
    binaries = tmp_path / "bin"
    binaries.mkdir()
    # Act as SSH's remote shell. Both explicit SSH and rsync use this transport.
    fake_ssh = binaries / "ssh"
    fake_ssh.write_text('''#!/usr/bin/python3
import os, subprocess, sys
args = sys.argv[1:]
while args and args[0].startswith('-'):
    flag = args.pop(0)
    if flag == '-o': args.pop(0)
assert args.pop(0) == 'local-test'
sys.exit(subprocess.call(' '.join(args), shell=True, executable='/bin/bash'))
''')
    fake_ssh.chmod(0o755)
    monkeypatch.setenv("PATH", str(binaries) + ":" + os.environ["PATH"])
    monkeypatch.setattr(remote_dev, "ROOT", source)

    def invoke(*args):
        return remote_dev.main(["--config", str(config), "robot", *args])

    return source, destination, invoke


def test_sync_preview_delete_and_protected_files(workspace):
    source, destination, invoke = workspace
    assert invoke("sync") == 1  # No unmarked directory is changed.
    assert not destination.exists()
    assert invoke("check") == 0
    assert invoke("init") == 0
    assert invoke("init") == 0
    for directory in [".git", ".deps", "ros2_ws/build", "ros2_ws/install", "ros2_ws/log"]:
        (source / directory).mkdir(parents=True)
        (source / directory / "local-only").write_text("private")
    (source / "deploy/targets.local.json").write_text("private")
    (source / ".env").write_text("private")
    (source / "outside-link").symlink_to("/etc/passwd")
    assert invoke("sync", "--dry-run") == 0
    assert not (destination / "hello.py").exists()
    assert invoke("sync") == 0
    assert (destination / "hello.py").read_text() == "version = 1\n"
    assert not (destination / ".git").exists()
    assert not (destination / "outside-link").exists()
    assert not (destination / "deploy/targets.local.json").exists()
    assert not (destination / ".env").exists()
    (destination / "ros2_ws/install").mkdir(parents=True)
    (destination / "ros2_ws/install/artifact").write_text("keep")
    (destination / "stale.py").write_text("old")
    assert invoke("sync") == 0
    assert (destination / "stale.py").exists()
    assert invoke("sync", "--delete", "--dry-run") == 0
    assert (destination / "stale.py").exists()
    assert invoke("sync", "--delete") == 0
    assert not (destination / "stale.py").exists()
    assert (destination / "ros2_ws/install/artifact").read_text() == "keep"
    assert (destination / remote_dev.MARKER).is_file()


def test_refuse_existing_directory_and_git_checkout(workspace):
    _, destination, invoke = workspace
    destination.mkdir(parents=True)
    (destination / "existing").write_text("keep")
    assert invoke("init") == 1
    assert not (destination / remote_dev.MARKER).exists()
    (destination / "existing").unlink()
    assert invoke("init") == 0
    (destination / ".git").mkdir()
    assert invoke("sync", "--delete") == 1
    assert invoke("build") == 1


def test_config_validation_and_remote_build_arguments(workspace, tmp_path, monkeypatch):
    _, _, invoke = workspace
    invalid = tmp_path / "invalid.json"
    for host, path in [("-oProxyCommand=bad", "/home/robot/dev"),
                       ("robot", "/"), ("robot", "/home/robot/../dev"),
                       ("robot", "/home/robot/$(bad)")]:
        invalid.write_text(json.dumps({"robot": {"host": host, "path": path}}))
        with pytest.raises(ValueError):
            remote_dev.target_config(invalid, "robot")
    assert invoke("build", "--packages", "bad;command") == 1
    calls = []
    monkeypatch.setattr(remote_dev, "ssh", lambda cfg, script, *args: calls.append((script, args)))
    assert invoke("build", "--packages", "hei_sensors", "hei_agent_bridge") == 0
    script, args = calls[0]
    assert args == ("jazzy", "", "hei_sensors", "hei_agent_bridge")
    assert 'args=(--packages-up-to "$@")' in script
    assert '"${args[@]}"' in script
    assert "--symlink-install" not in script
    for setup in ["relative/setup.bash", "~/../setup.bash", "/tmp/$(bad)"]:
        invalid.write_text(json.dumps({"robot": {"host": "robot", "path": "/home/robot/dev", "ros_setup": setup}}))
        with pytest.raises(ValueError):
            remote_dev.target_config(invalid, "robot")


def test_remote_build_execution_and_failure(workspace, tmp_path, monkeypatch):
    source, destination, invoke = workspace
    (source / "scripts/env").mkdir(parents=True)
    shutil.copy(REPO / "scripts/env/workspace.bash", source / "scripts/env/workspace.bash")
    (source / "scripts/env/ros-rust.bash").write_text("export HEI_TEST_RUST=ready\n")
    (source / "scripts/env/agent-bridge.bash").write_text("export HEI_TEST_AGENT=ready\n")
    ros = tmp_path / "ros/jazzy"
    ros.mkdir(parents=True)
    (ros / "setup.bash").write_text("export ROS_DISTRO=jazzy\n")
    # Only the ROS installation is substituted; execute the actual remote build script.
    monkeypatch.setattr(remote_dev, "BUILD", remote_dev.BUILD.replace("/opt/ros/", str(tmp_path / "ros") + "/"))
    colcon = tmp_path / "bin/colcon"
    colcon.write_text('''#!/usr/bin/python3
import json, os, pathlib, sys
assert os.environ['ROS_DISTRO'] == 'jazzy'
assert os.environ['HEI_TEST_RUST'] == os.environ['HEI_TEST_AGENT'] == 'ready'
pathlib.Path('build-args.json').write_text(json.dumps(sys.argv[1:]))
sys.exit(7 if pathlib.Path('fail-build').exists() else 0)
''')
    colcon.chmod(0o755)
    assert invoke("init") == 0
    assert invoke("sync") == 0
    assert invoke("build", "--packages", "hei_sensors") == 0
    args = json.loads((destination / "build-args.json").read_text())
    assert args[-2:] == ["--packages-up-to", "hei_sensors"]
    # A source-built dependency overlay may be the selected ROS setup.
    overlay = tmp_path / "overlay/setup.bash"
    overlay.parent.mkdir()
    overlay.write_text("export ROS_DISTRO=jazzy\nprintf loaded > overlay-loaded\n")
    config = tmp_path / "targets.json"
    targets = json.loads(config.read_text())
    targets["robot"]["ros_setup"] = str(overlay)
    config.write_text(json.dumps(targets))
    assert invoke("build", "--packages", "hei_sensors") == 0
    assert (destination / "overlay-loaded").read_text() == "loaded"
    (destination / "fail-build").touch()
    assert invoke("build") == 1


def test_watch_coalesces_edits_and_propagates_deletion(workspace, monkeypatch):
    source, destination, invoke = workspace
    assert invoke("init") == 0
    tick = [0]
    transfers = []
    real_sync = remote_dev.sync_source

    def sync(cfg, **kwargs):
        transfers.append(tick[0])
        real_sync(cfg, **kwargs)

    def sleep(_):
        tick[0] += 1
        if tick[0] in (1, 2):
            # Same-size updates; checksum sync must not rely on rsync's mtime resolution.
            edited = source / "hello.py"
            edited.write_text(f"version = {tick[0] + 1}\n")
            # The fake clock must also advance file metadata: some filesystems
            # give rapid consecutive writes the same mtime/ctime.
            stamp = tick[0] * 1_000_000_000
            os.utime(edited, ns=(stamp, stamp))
        elif tick[0] == 5:
            assert (destination / "hello.py").read_text() == "version = 3\n"
            (source / "hello.py").unlink()
        elif tick[0] == 8:
            assert not (destination / "hello.py").exists()
            raise KeyboardInterrupt

    monkeypatch.setattr(remote_dev, "sync_source", sync)
    monkeypatch.setattr(remote_dev.time, "sleep", sleep)
    monkeypatch.setattr(remote_dev.time, "monotonic", lambda: tick[0] / 10)
    assert invoke("watch", "--delete", "--interval", ".1", "--debounce", ".15") == 0
    assert transfers == [0, 4, 7]


def test_watch_ignores_excluded_changes_and_checks_options(workspace):
    source, _, invoke = workspace
    before = remote_dev.source_snapshot()
    (source / "build").mkdir()
    (source / "build/output").write_text("generated")
    (source / "deploy/targets.local.json").write_text("private")
    assert remote_dev.source_snapshot() == before
    for value in ["0", "-1", "nan", "inf"]:
        assert invoke("watch", "--interval", value) == 1


def test_home_relative_path_resolution(tmp_path, monkeypatch):
    config = tmp_path / "targets.json"
    config.write_text(json.dumps({"robot": {"host": "seeed@seeed.local", "path": "~/gitdir/easeBot"}}))
    cfg = remote_dev.target_config(config, "robot")
    calls = []

    def query(cmd, **kwargs):
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 0, stdout="/home/seeed\n")

    monkeypatch.setattr(remote_dev.subprocess, "run", query)
    assert remote_dev.resolve_home(cfg)["path"] == "/home/seeed/gitdir/easeBot"
    assert calls[0][-1] == "printf '%s\\n' \"$HOME\""
    assert calls[0][-2] == "seeed@seeed.local"
    for path in ["~/..", "~/gitdir/../../etc", "~other/gitdir/easeBot", "~/"]:
        config.write_text(json.dumps({"robot": {"host": "robot", "path": path}}))
        with pytest.raises(ValueError):
            remote_dev.target_config(config, "robot")
    monkeypatch.setattr(remote_dev.subprocess, "run",
                        lambda *args, **kwargs: subprocess.CompletedProcess(args, 0, stdout="banner\n/home/seeed\n"))
    with pytest.raises(ValueError, match="remote HOME"):
        remote_dev.resolve_home(cfg)


def test_remote_setup_uses_configured_underlay_and_same_entry(workspace, tmp_path, monkeypatch):
    source, destination, invoke = workspace
    (source / 'scripts/env').mkdir(parents=True)
    (source / 'scripts/env/workspace.bash').write_text('export HEI_SETUP_ENV=loaded\n')
    (source / 'scripts/setup.bash').write_text('''set -e
[[ "$ROS_DISTRO" == jazzy && "$HEI_SETUP_ENV" == loaded ]]
printf '%s\\n' "$@" > setup-args.txt
''')
    overlay = tmp_path / 'overlay/setup.bash'
    overlay.parent.mkdir()
    overlay.write_text('export ROS_DISTRO=jazzy\n')
    cfg = tmp_path / 'setup-targets.json'
    cfg.write_text(json.dumps({'robot': {'host': 'local-test', 'path': str(destination),
                                         'ros_setup': str(overlay)}}))
    assert invoke('init') == 0
    assert invoke('sync') == 0
    assert remote_dev.main(['--config', str(cfg), 'robot', 'setup', '--without-agent-bridge']) == 0
    assert (destination / 'setup-args.txt').read_text().strip() == '--without-agent-bridge'
    (destination / '.git').mkdir()
    assert remote_dev.main(['--config', str(cfg), 'robot', 'setup']) == 1
