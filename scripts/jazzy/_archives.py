"""Shared archive cache operations for the Jazzy source preparation tools."""
import hashlib
from pathlib import Path
import shutil
import urllib.parse
import urllib.request


def github_archive_url(repository, ref):
    project = repository.removeprefix("https://github.com/").removesuffix(".git")
    return f"https://codeload.github.com/{project}/tar.gz/{urllib.parse.quote(ref, safe='')}"


def download(url, target):
    """Publish completed downloads atomically; interrupted files are not cached."""
    target = Path(target)
    partial = target.with_suffix(target.suffix + ".part")
    request = urllib.request.Request(url, headers={"User-Agent": "hei-jazzy-source-build"})
    try:
        with urllib.request.urlopen(request, timeout=90) as response, partial.open("wb") as stream:
            shutil.copyfileobj(response, stream)
        partial.replace(target)
    finally:
        partial.unlink(missing_ok=True)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
