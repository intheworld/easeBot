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


def setup_workspace(tmp_path, monkeypatch):
    """Run the real entry point; replace only installers, never touch ROS/network."""
    import os
    import shutil
    repo = tmp_path / 'repo'
    (repo / 'scripts/setup').mkdir(parents=True)
    (repo / 'scripts/env').mkdir()
    shutil.copy(REPO / 'scripts/setup.bash', repo / 'scripts/setup.bash')
    (repo / 'scripts/env/workspace.bash').write_text(':\n')
    trace = tmp_path / 'steps'
    monkeypatch.setenv('HEI_SETUP_TRACE', str(trace))
    for name in ('prepare-build-tools.py', 'prepare-rust-ros.py', 'prepare-sensors.py'):
        (repo / 'scripts/setup' / name).write_text(
            'import os, pathlib, sys\n'
            'name = pathlib.Path(__file__).name\n'
            'with open(os.environ["HEI_SETUP_TRACE"], "a") as f: f.write(name + "\\n")\n'
            'sys.exit(9 if os.environ.get("HEI_SETUP_FAIL") == name else 0)\n')
    for name in ('prepare-motorbridge.bash', 'prepare-agent-bridge.sh'):
        (repo / 'scripts/setup' / name).write_text(
            'printf "%s\\n" "${BASH_SOURCE[0]##*/}" >> "$HEI_SETUP_TRACE"\n')
    prefix = tmp_path / 'jazzy'
    for name in ('sensor_msgs', 'nav_msgs', 'diagnostic_msgs'):
        binding = prefix / 'share' / name / 'rust/Cargo.toml'
        binding.parent.mkdir(parents=True)
        binding.touch()
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    monkeypatch.setenv('AMENT_PREFIX_PATH', str(prefix))
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    for name in ('git', 'cmake', 'c++', 'cargo', 'rustc', 'colcon'):
        tool = binaries / name
        tool.write_text('#!/bin/sh\nexit 0\n')
        tool.chmod(0o755)
    monkeypatch.setenv('PATH', str(binaries) + os.pathsep + os.environ['PATH'])
    return repo, prefix, trace


@pytest.mark.parametrize('skip_agent', [False, True])
def test_setup_runs_from_any_directory_and_selects_optional_sdk(tmp_path, monkeypatch, skip_agent):
    import subprocess
    repo, _, trace = setup_workspace(tmp_path, monkeypatch)
    command = ['bash', str(repo / 'scripts/setup.bash')]
    if skip_agent:
        command.append('--without-agent-bridge')
    subprocess.run(command, cwd=tmp_path, check=True)
    expected = ['prepare-build-tools.py', 'prepare-rust-ros.py', 'prepare-motorbridge.bash', 'prepare-sensors.py']
    if not skip_agent:
        expected.append('prepare-agent-bridge.sh')
    assert trace.read_text().splitlines() == expected


def test_setup_rejects_invalid_environment_before_any_install(tmp_path, monkeypatch):
    import subprocess
    repo, prefix, trace = setup_workspace(tmp_path, monkeypatch)
    command = ['bash', str(repo / 'scripts/setup.bash')]
    monkeypatch.setenv('ROS_DISTRO', 'humble')
    assert subprocess.run(command).returncode == 1
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    (prefix / 'share/nav_msgs/rust/Cargo.toml').unlink()
    assert subprocess.run(command).returncode == 1
    assert not trace.exists()
    assert subprocess.run(command + ['--unknown']).returncode == 2
    assert subprocess.run(command + ['--help']).returncode == 0
    assert not trace.exists()


def test_setup_stops_at_failed_dependency(tmp_path, monkeypatch):
    import subprocess
    repo, _, trace = setup_workspace(tmp_path, monkeypatch)
    monkeypatch.setenv('HEI_SETUP_FAIL', 'prepare-rust-ros.py')
    assert subprocess.run(['bash', str(repo / 'scripts/setup.bash')]).returncode == 9
    assert trace.read_text().splitlines() == ['prepare-build-tools.py', 'prepare-rust-ros.py']


def test_server_setup_needs_no_ros_and_installs_both_local_packages(tmp_path, monkeypatch):
    import subprocess
    repo, _, trace = setup_workspace(tmp_path, monkeypatch)
    monkeypatch.delenv('ROS_DISTRO', raising=False)
    monkeypatch.delenv('AMENT_PREFIX_PATH', raising=False)
    python = repo / 'agent/.venv/bin/python'
    python.parent.mkdir(parents=True)
    python.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$HEI_SETUP_TRACE"\n')
    python.chmod(0o755)
    command = ['bash', str(repo / 'scripts/setup.bash'), '--agent-server']
    subprocess.run(command, cwd=tmp_path, check=True)
    assert trace.read_text().splitlines() == [
        '-m', 'pip', 'install', '-e', str(repo / 'packages/hei-discovery'),
        '-e', str(repo / 'agent') + '[llm,mhs]']
    trace.unlink()
    assert subprocess.run(command + ['--without-agent-bridge']).returncode == 2
    assert not trace.exists()


def test_workspace_pins_native_cmake_despite_broken_path_launcher(tmp_path, monkeypatch):
    import os
    import subprocess
    launcher = tmp_path / 'cmake'
    launcher.write_text('#!/bin/sh\nexit 77\n')
    launcher.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path) + os.pathsep + os.environ['PATH'])
    monkeypatch.setenv('PYTHONNOUSERSITE', '1')
    monkeypatch.delenv('CMAKE_COMMAND', raising=False)
    monkeypatch.delenv('CTEST_COMMAND', raising=False)
    source = 'source "$1/scripts/env/workspace.bash"\n'
    result = subprocess.run(['bash', '-c', source +
        'test "$CMAKE_COMMAND" = /usr/bin/cmake && test "$CTEST_COMMAND" = /usr/bin/ctest && "$CMAKE_COMMAND" --version',
        'test', str(REPO)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    monkeypatch.setenv('CMAKE_COMMAND', str(launcher))
    result = subprocess.run(['bash', '-c', source + 'test "$CMAKE_COMMAND" = "$2"',
        'test', str(REPO), str(launcher)])
    assert result.returncode == 0  # Explicit caller overrides are preserved.


def test_camera_sdk_preparation_stays_in_workspace(tmp_path, monkeypatch):
    module = load_script('prepare_sensors', 'scripts/setup/prepare-sensors.py')
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(module.sys, 'argv', ['prepare-sensors.py'])
    monkeypatch.setattr(module.metadata, 'distributions', lambda **kw: [])
    monkeypatch.setattr(module.shutil, 'which', lambda name: '/usr/bin/uv')
    calls = []
    monkeypatch.setattr(module.subprocess, 'run', lambda args, **kw: calls.append((args, kw)))
    module.main()
    args, kwargs = calls[0]
    assert args[args.index('--target') + 1] == str(tmp_path / '.deps/sensors-python')
    assert args[-1] == 'pyrealsense2==' + module.VERSION and kwargs['check']


def test_build_plugin_installer_uses_workspace_and_reuses_existing_tools(tmp_path, monkeypatch):
    module = load_script('prepare_build_tools', 'scripts/setup/prepare-build-tools.py')
    monkeypatch.setattr(module, 'ROOT', tmp_path)
    monkeypatch.setattr(module.sys, 'argv', ['prepare-build-tools.py'])
    calls = []
    monkeypatch.setattr(module.subprocess, 'run', lambda args, **kw: calls.append((args, kw)))
    monkeypatch.setattr(module, 'available', lambda: True)
    module.main()
    assert calls == []
    monkeypatch.setattr(module, 'available', lambda: False)
    monkeypatch.setattr(module.shutil, 'which', lambda name: '/usr/bin/uv')
    module.main()
    args, kwargs = calls[0]
    assert args[args.index('--target') + 1] == str(tmp_path / '.deps/build-python')
    assert '--system' not in args and '--user' not in args
    assert kwargs['check']


@pytest.mark.parametrize('fetch_succeeds', [True, False])
def test_sdk_archive_migration_preserves_old_copy_on_failure(tmp_path, monkeypatch, fetch_succeeds):
    import os
    import shutil
    import subprocess
    upstream = tmp_path / 'upstream'
    upstream.mkdir()
    def git(*args):
        return subprocess.check_output(['git', '-C', str(upstream), *args], text=True).strip()
    git('init', '-q')
    for name in ('python', 'schema'):
        path = upstream / 'packages/mhs' / name / 'keep'
        path.parent.mkdir(parents=True)
        path.write_text('pinned SDK\n')
    git('add', '.')
    git('-c', 'user.name=Test', '-c', 'user.email=test@example.invalid', 'commit', '-qm', 'fixture')
    revision = git('rev-parse', 'HEAD')
    repo = tmp_path / 'repo'
    script = repo / 'scripts/setup/prepare-agent-bridge.sh'
    script.parent.mkdir(parents=True)
    shutil.copy(REPO / 'scripts/setup/prepare-agent-bridge.sh', script)
    package = repo / 'ros2_ws/src/hei_agent_bridge'
    package.mkdir(parents=True)
    (package / 'sdk-revision.txt').write_text(revision)
    (package / 'requirements-mhs.lock').touch()
    sdk = repo / '.deps/agnes-harness'
    sdk.mkdir(parents=True)
    (sdk / 'local-copy').write_text('preserve me')
    binary = tmp_path / 'bin'
    binary.mkdir()
    uv = binary / 'uv'
    uv.write_text('#!/bin/sh\nexit 0\n')
    uv.chmod(0o755)
    monkeypatch.setenv('PATH', str(binary) + os.pathsep + os.environ['PATH'])
    remote = upstream if fetch_succeeds else tmp_path / 'missing-upstream'
    monkeypatch.setenv('GIT_CONFIG_COUNT', '1')
    monkeypatch.setenv('GIT_CONFIG_KEY_0', f'url.{remote.as_uri()}.insteadOf')
    monkeypatch.setenv('GIT_CONFIG_VALUE_0', 'https://github.com/AgnesAI-Labs/agnes-harness.git')
    result = subprocess.run(['bash', str(script)], cwd=tmp_path)
    if fetch_succeeds:
        assert result.returncode == 0
        assert (sdk / 'packages/mhs/python/keep').read_text() == 'pinned SDK\n'
        backup = list((repo / '.deps').glob('agnes-harness-before-setup.*/source/local-copy'))
        assert len(backup) == 1 and backup[0].read_text() == 'preserve me'
        subprocess.run(['bash', str(script)], check=True)
        assert len(list((repo / '.deps').glob('agnes-harness-before-setup.*'))) == 1
    else:
        assert result.returncode != 0
        assert (sdk / 'local-copy').read_text() == 'preserve me'
        assert not list((repo / '.deps').glob('agnes-harness-before-setup.*'))
    assert not list((repo / '.deps').glob('agnes-harness.prepare.*'))


@pytest.mark.parametrize('layout', ['install/hei_sensors', 'install'])
def test_camera_launch_resolves_local_sdk_without_fresh_shell(tmp_path, monkeypatch, layout):
    pytest.importorskip('launch_ros')
    import runpy
    launch = runpy.run_path(str(REPO / 'ros2_ws/src/hei_bringup/launch/cameras.launch.py'))
    sdk = tmp_path / '.deps/sensors-python'
    sdk.mkdir(parents=True)
    prefix = tmp_path / 'ros2_ws' / layout
    prefix.mkdir(parents=True)
    environment = launch['camera_environment']
    monkeypatch.setitem(environment.__globals__, 'get_package_prefix', lambda _: str(prefix))
    monkeypatch.setenv('PYTHONPATH', '/existing/ros/python')
    assert environment() == {'PYTHONNOUSERSITE':'1', 'PYTHONPATH':str(sdk)+':/existing/ros/python'}


def test_arm_compute_check_does_not_require_detector_or_weights(tmp_path, monkeypatch):
    import sys
    script=load_script('prepare_grasp_check','scripts/setup/prepare-grasp.py')
    calls=[]
    monkeypatch.setattr(sys,'argv',['prepare-grasp.py','--arm-only','--python',sys.executable,
                                 '--model',str(tmp_path/'missing-model.pt')])
    monkeypatch.setattr(script.subprocess,'run',lambda command,**kwargs:calls.append(command))
    script.main()
    assert len(calls)==1 and 'mujoco' in calls[0][-1] and 'scipy' in calls[0][-1]
    assert 'ultralytics' not in calls[0][-1]
