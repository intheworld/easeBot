#!/usr/bin/env python3
"""Cache vendor archives and record download-only CMake patches for Jazzy."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import re
import tarfile

from _archives import download, github_archive_url, sha256


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    workspace = Path.home() / 'ros2_jazzy'
    cache = workspace / 'downloads'
    cache.mkdir(parents=True, exist_ok=True)
    lock_path = workspace / 'vendor-archives.lock.json'
    lock = json.loads(lock_path.read_text()) if lock_path.exists() else {}

    def archive(repository, ref):
        url = github_archive_url(repository, ref)
        target = cache / ('vendor-' + hashlib.sha256(url.encode()).hexdigest()[:20] + '.tar.gz')
        if not target.exists():
            download(url, target)
        with tarfile.open(target) as bundle:
            prefix = bundle.getmembers()[0].name.split('/')[0]
        digest = sha256(target)
        if url in lock and lock[url]['sha256'] != digest:
            raise ValueError(f'cached vendor archive hash mismatch: {target}')
        lock[url] = {'sha256': digest, 'archive': str(target), 'prefix': prefix}
        lock_path.write_text(json.dumps(lock, indent=2, sort_keys=True) + '\n')
        return target, prefix, digest

    patch_parts = []
    for cmake in sorted((workspace / 'src').rglob('CMakeLists.txt')):
        if not any('vendor' in part for part in cmake.relative_to(workspace / 'src').parts):
            continue
        before = cmake.read_text()

        def external(match):
            repository, ref = match.groups()
            target, _, digest = archive(repository, ref)
            return f'URL {target.as_uri()}\n  URL_HASH SHA256={digest}'

        def ament(match):
            repository, ref = match.groups()
            target, prefix, _ = archive(repository, ref)
            return f'VCS_TYPE tar\n  VCS_URL {target.as_uri()}\n  VCS_VERSION {prefix}'

        after = re.sub(r'GIT_REPOSITORY (https://github.com/[^\s]+\.git)\s+GIT_TAG ([A-Za-z0-9_./-]+)(?=\s|$)', external, before)
        after = re.sub(r'(?:VCS_TYPE git\s+)?VCS_URL (https://github.com/[^\s]+\.git)\s+VCS_VERSION ([A-Za-z0-9_./-]+)(?=\s|$)', ament, after)
        if after != before:
            original = cmake.with_name('CMakeLists.txt.before-jazzy-archives')
            if not original.exists():
                original.write_text(before)
            cmake.write_text(after)
            print('Prepared vendor download:', cmake.relative_to(workspace), flush=True)
        original = cmake.with_name('CMakeLists.txt.before-jazzy-archives')
        if original.exists():
            name = str(cmake.relative_to(workspace))
            patch_parts.extend(difflib.unified_diff(original.read_text().splitlines(True),
                                                  cmake.read_text().splitlines(True),
                                                  fromfile='a/' + name, tofile='b/' + name))
    (workspace / 'vendor-downloads.patch').write_text(''.join(patch_parts))


if __name__ == '__main__':
    main()
