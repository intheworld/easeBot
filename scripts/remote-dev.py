#!/usr/bin/env python3
"""Push a development checkout over SSH and build on the destination."""
import argparse
import fnmatch
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MARKER = ".hei-sync-root"
REMOTE_SCRIPTS = Path(__file__).resolve().parent / "remote"
GUARD = (REMOTE_SCRIPTS / "guard.bash").read_text()
INIT = (REMOTE_SCRIPTS / "init.bash").read_text()
BUILD = GUARD + (REMOTE_SCRIPTS / "build.bash").read_text()


def target_config(path, name):
    targets = json.loads(path.read_text())
    if not isinstance(targets, dict):
        raise ValueError("config must be a JSON object of named targets")
    cfg = targets[name]
    if not isinstance(cfg, dict):
        raise ValueError("each target must be a JSON object")
    host, remote = cfg["host"], cfg["path"]
    # A simple SSH alias/user@host; ports, keys and jump hosts belong in SSH config.
    if not isinstance(host, str) or not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@-]*", host):
        raise ValueError("host must be an SSH alias or user@hostname (use ~/.ssh/config for options)")
    if not isinstance(remote, str) or not re.fullmatch(r"(?:/|~/)[A-Za-z0-9_./-]+", remote):
        raise ValueError("path must start with / or ~/, without spaces or shell metacharacters")
    home_relative = remote.startswith("~/")
    path_parts = PurePosixPath(remote[2:] if home_relative else remote)
    if ".." in path_parts.parts or len(path_parts.parts) < (2 if home_relative else 4):
        raise ValueError("use a dedicated path at least two levels below ~/ or three below /, without '..'")
    cfg = dict(cfg, path=("~/" if home_relative else "") + str(path_parts))
    distro = cfg.get("ros_distro", "jazzy")
    if not isinstance(distro, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", distro):
        raise ValueError("invalid ros_distro")
    cfg["ros_distro"] = distro
    setup = cfg.get("ros_setup", "")
    if (not isinstance(setup, str) or (setup and (
            not re.fullmatch(r"(?:/|~/)[A-Za-z0-9_./-]+", setup)
            or ".." in PurePosixPath(setup).parts))):
        raise ValueError("ros_setup must be an absolute or ~/ path without shell metacharacters")
    cfg["ros_setup"] = setup
    return cfg


def resolve_home(cfg):
    """Resolve ~ on the destination, never against the local user's home."""
    if not cfg["path"].startswith("~/"):
        return cfg
    options = ["-o", "BatchMode=yes"] if cfg.get("batch_mode") else []
    cmd = ["ssh", "-T", "-o", "ConnectTimeout=10", *options, cfg["host"],
           "printf '%s\\n' \"$HOME\""]
    print("+ " + shlex.join(cmd), flush=True)
    home = subprocess.run(cmd, check=True, text=True, stdout=subprocess.PIPE).stdout.rstrip("\n")
    if (not re.fullmatch(r"/[A-Za-z0-9_./-]+", home)
            or ".." in PurePosixPath(home).parts):
        raise ValueError("remote HOME must be a single absolute path without spaces or shell metacharacters")
    resolved = str(PurePosixPath(home) / cfg["path"][2:])
    if len(PurePosixPath(resolved).parts) < 4:
        raise ValueError("resolved path is too shallow; choose a dedicated development directory")
    print(f"Remote directory: {resolved}", flush=True)
    return dict(cfg, path=resolved)


def run(argv, **kwargs):
    print("+ " + shlex.join(argv), flush=True)
    subprocess.run(argv, check=True, **kwargs)


def ssh(cfg, script, *args):
    command = shlex.join(["bash", "-s", "--", cfg["path"], *args])
    options = ["-o", "BatchMode=yes"] if cfg.get("batch_mode") else []
    run(["ssh", "-T", "-o", "ConnectTimeout=10", *options, cfg["host"], command], input=script, text=True)


def sync_source(cfg, *, dry_run=False, delete=False, checksum=False):
    if not shutil.which("rsync"):
        raise ValueError("install local rsync first")
    ssh(cfg, GUARD)
    transport = "ssh -o ConnectTimeout=10"
    if cfg.get("batch_mode"):
        transport += " -o BatchMode=yes"
    cmd = ["rsync", "-az", "--itemize-changes", "--safe-links",
           "--exclude-from", str(ROOT / "deploy/rsync.exclude"), "-e", transport]
    if dry_run:
        cmd.append("--dry-run")
    if delete:
        cmd.append("--delete-delay")
    if checksum:
        cmd.append("--checksum")
    cmd += [str(ROOT) + "/", f"{cfg['host']}:{cfg['path']}/"]
    run(cmd)


def source_snapshot():
    """Match our basename-only rsync exclusion rules without following symlinks."""
    patterns = [line.strip() for line in (ROOT / "deploy/rsync.exclude").read_text().splitlines()
                if line.strip() and not line.lstrip().startswith("#")]
    if any("/" in pattern.rstrip("/") for pattern in patterns):
        raise ValueError("watch currently supports only basename exclude patterns, optionally ending in /")

    def excluded(name, directory):
        return any(fnmatch.fnmatchcase(name, pattern.rstrip("/"))
                   for pattern in patterns if directory or not pattern.endswith("/"))

    snapshot = {}

    def scan_error(error):
        raise error

    for parent, dirs, files in os.walk(ROOT, onerror=scan_error, followlinks=False):
        dirs[:] = [name for name in dirs if not excluded(name, True)]
        for name in dirs + [name for name in files if not excluded(name, False)]:
            path = Path(parent) / name
            try:
                stat = path.lstat()
                key = str(path.relative_to(ROOT))
                if path.is_symlink():
                    snapshot[key] = (stat.st_mode, os.readlink(path))
                elif name in dirs:
                    # Cache activity in an excluded child must not trigger a sync.
                    snapshot[key] = (stat.st_mode,)
                else:
                    snapshot[key] = (stat.st_mode, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
            except FileNotFoundError:
                # Editors may atomically rename a file while a scan is in progress.
                continue
    return snapshot


def watch_source(cfg, *, delete=False, interval=0.5, debounce=1.0):
    if not all(math.isfinite(value) and value > 0 for value in (interval, debounce)):
        raise ValueError("interval and debounce must be finite positive seconds")
    cfg = dict(cfg, batch_mode=True)
    observed = source_snapshot()
    sync_source(cfg, delete=delete, checksum=True)
    synced = observed
    changed_at = time.monotonic()
    print("Watching local source; Ctrl+C stops. No automatic build or restart.", flush=True)
    while True:
        time.sleep(interval)
        current = source_snapshot()
        now = time.monotonic()
        if current != observed:
            observed = current
            changed_at = now
        if observed != synced and now - changed_at >= debounce:
            sync_source(cfg, delete=delete, checksum=True)
            synced = observed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "deploy/targets.local.json")
    parser.add_argument("target", help="target name from the local JSON config")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check", help="check SSH access and remote rsync/bash")
    commands.add_parser("init", help="create/mark a new or empty remote development directory")
    sync = commands.add_parser("sync", help="push source files; never starts or stops nodes")
    sync.add_argument("--dry-run", action="store_true")
    sync.add_argument("--delete", action="store_true", help="also remove remote source files absent locally")
    watch = commands.add_parser("watch", help="initial sync, then push local changes until Ctrl+C")
    watch.add_argument("--delete", action="store_true", help="also propagate source deletions")
    watch.add_argument("--interval", type=float, default=0.5, help="local scan interval in seconds (default .5)")
    watch.add_argument("--debounce", type=float, default=1.0, help="quiet time before syncing (default 1)")
    build = commands.add_parser("build", help="build the previously synced tree on the remote machine")
    build.add_argument("--packages", nargs="+", default=[], help="packages and recursive dependencies; default all")
    args = parser.parse_args(argv)
    try:
        cfg = target_config(args.config, args.target)
        if not shutil.which("ssh"):
            raise ValueError("install the local OpenSSH client first")
        if args.command == "watch":
            cfg = dict(cfg, batch_mode=True)
        cfg = resolve_home(cfg)
        if args.command == "check":
            ssh(cfg, "set -e\ncommand -v bash\ncommand -v rsync\nuname -sm\nprintf 'Target directory: %s\\n' \"$1\"\n")
        elif args.command == "init":
            ssh(cfg, INIT)
        elif args.command == "sync":
            sync_source(cfg, dry_run=args.dry_run, delete=args.delete)
        elif args.command == "watch":
            watch_source(cfg, delete=args.delete, interval=args.interval, debounce=args.debounce)
        elif args.command == "build":
            if any(not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_]*", p) for p in args.packages):
                raise ValueError("invalid ROS package name")
            ssh(cfg, BUILD, cfg["ros_distro"], cfg["ros_setup"], *args.packages)
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        return 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        print(f"remote-dev: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
