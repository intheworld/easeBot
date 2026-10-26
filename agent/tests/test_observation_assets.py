"""Bounded observation transfer, no ROS/numpy or arbitrary URL fetching."""
import asyncio
import base64
import hashlib
from pathlib import Path

import pytest
from hei_agent.mhs import MHSConnector


class Assets:
    def __init__(self, corrupt=False):
        self.corrupt = corrupt
        self.calls = []

    async def invoke(self, device, tool, arguments, **kwargs):
        self.calls.append((tool, arguments))
        assert tool == 'read_observation_asset'
        data = b'jpeg-test' if arguments['kind'] == 'rgb' else b'{}'
        return {'status': 'done', 'data': {'mime_type': 'image/jpeg' if arguments['kind'] == 'rgb' else 'application/json',
            'data_base64': base64.b64encode(data).decode(), 'byte_length': len(data),
            'sha256': 'bad' if self.corrupt else hashlib.sha256(data).hexdigest()}}


def outputs():
    identity = 'a' * 32
    return {'observation_id': identity, 'rgb_ref': f'mhs-observation:{identity}/rgb',
            'depth_ref': '', 'depth_available': False, 'metadata_ref': f'mhs-observation:{identity}/metadata'}


def test_downloaded_assets_use_local_paths_and_keep_snapshot_identity(tmp_path):
    hub = Assets()
    result = asyncio.run(MHSConnector(hub, 'robot', artifact_dir=tmp_path)._observation_assets(outputs()))
    assert result['observation_id'] == 'a' * 32
    assert Path(result['rgb_ref']).read_bytes() == b'jpeg-test'
    assert Path(result['metadata_ref']).read_bytes() == b'{}'
    assert result['depth_ref'] == '' and len(hub.calls) == 2


def test_corrupt_asset_is_not_persisted(tmp_path):
    with pytest.raises(RuntimeError, match='integrity'):
        asyncio.run(MHSConnector(Assets(corrupt=True), 'robot', artifact_dir=tmp_path)._observation_assets(outputs()))
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('change', [{'observation_id': '../escape'}, {'rgb_ref': 'https://example.com/image.jpg'}])
def test_remote_paths_and_urls_are_never_followed(tmp_path, change):
    hub = Assets()
    with pytest.raises(RuntimeError):
        asyncio.run(MHSConnector(hub, 'robot', artifact_dir=tmp_path)._observation_assets({**outputs(), **change}))
    assert not hub.calls and not list(tmp_path.iterdir())
