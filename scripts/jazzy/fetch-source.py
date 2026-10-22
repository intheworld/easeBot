#!/usr/bin/env python3
"""Fetch official Jazzy source archives when Git transport is unavailable.

Keeps archives and their SHA256 hashes for reproducibility. No system changes.
"""
import argparse
import concurrent.futures
import inspect
import json
from pathlib import Path, PurePosixPath
import tarfile
import tempfile

import yaml

from _archives import download, github_archive_url, sha256


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    workspace = Path.home() / 'ros2_jazzy'
    workspace.mkdir(exist_ok=True)
    cache = workspace / 'downloads'
    cache.mkdir(exist_ok=True)
    manifest = workspace / 'ros2.repos'
    if not manifest.exists():
        download('https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos', manifest)
    repos = yaml.safe_load(manifest.read_text())['repositories']
    repos['ros2/variants'] = {'type': 'git', 'url': 'https://github.com/ros2/variants.git', 'version': 'jazzy'}
    lock_path = workspace / 'source-archives.lock.json'
    lock = json.loads(lock_path.read_text()) if lock_path.exists() else {}

    def fetch(item):
        name, repo = item
        relative = PurePosixPath(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError(f'invalid repository path: {name}')
        url = repo['url'].removesuffix('.git')
        if repo['type'] != 'git' or not url.startswith('https://github.com/'):
            raise ValueError(f'unsupported source: {repo}')
        archive_url = github_archive_url(url, repo['version'])
        archive = cache / (name.replace('/', '__') + '.tar.gz')
        destination = workspace / 'src' / name
        if name in lock and destination.is_dir():
            return name, lock[name]
        if not archive.exists():
            download(archive_url, archive)
        digest = sha256(archive)
        if name in lock and lock[name]['sha256'] != digest:
            raise ValueError(f'cached archive hash mismatch: {name}')
        if destination.exists():
            raise ValueError(f'refusing to overwrite unrecorded source: {destination}')
        destination.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='extract-', dir=cache) as staging:
            with tarfile.open(archive) as bundle:
                if 'filter' not in inspect.signature(bundle.extractall).parameters:
                    raise RuntimeError('Python tarfile data filter is required; update the system Python security patches')
                bundle.extractall(staging, filter='data')
            roots = list(Path(staging).iterdir())
            if len(roots) != 1 or not roots[0].is_dir():
                raise ValueError(f'unexpected archive layout: {name}')
            roots[0].rename(destination)
        return name, {'url': archive_url, 'sha256': digest, 'ref': repo['version']}

    failures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        futures = {pool.submit(fetch, item): item[0] for item in repos.items()}
        for future in concurrent.futures.as_completed(futures):
            try:
                name, record = future.result()
                lock[name] = record
                temporary = lock_path.with_suffix('.tmp')
                temporary.write_text(json.dumps(lock, indent=2, sort_keys=True) + '\n')
                temporary.replace(lock_path)
                print(f'[{len(lock)}/{len(repos)}] {name}', flush=True)
            except Exception as exc:
                failures.append(futures[future])
                print(f'FAILED {futures[future]}: {exc}', flush=True)
    if failures:
        raise SystemExit('Source downloads incomplete; rerun to resume: ' + ', '.join(failures))
    print(f'Sources ready: {workspace}/src', flush=True)


if __name__ == '__main__':
    main()
