"""Discovery adapter for the SDK revision in sdk-revision.txt.

SDK Device.run retries one fixed URL. These private hooks are isolated here and
tested against the pinned SDK. Keep the device, invocation ledger and lock alive
when returning to discovery; never recreate a SkillSet on transport loss.
"""
import asyncio
import logging

from agnes_mhs import Device
from agnes_mhs.device import Refused

log = logging.getLogger(__name__)


class _Rediscover(Exception):
    pass


class DiscoveryDevice(Device):
    async def run(self, url, *, hub_id='', interfaces=''):
        self._discovering = url == 'auto'
        self._refused = False
        if not self._discovering:
            return await super().run(url)
        try:
            from hei_discovery import HubDiscovery
        except ImportError as exc:
            raise RuntimeError('Missing discovery library: rerun scripts/setup/prepare-agent-bridge.sh') from exc
        async with HubDiscovery(hub_id=hub_id, device_id=self.id, interfaces=interfaces) as discovery:
            attempt, delay = 0, .5
            while True:
                endpoint = await discovery.wait()
                address = endpoint.urls[attempt % len(endpoint.urls)]
                attempt += 1
                self._finished = False  # SDK run() marks Nerve/deadman loops finished.
                self._refused = False
                log.info('Discovered MHS Hub %s at %s', endpoint.hub_id, address)
                try:
                    await super().run(address)
                    return  # Registration refusal/replacement remains terminal.
                except _Rediscover:
                    await asyncio.sleep(delay)
                    delay = min(delay * 2, 5)

    async def _register(self, ws):
        try:
            await super()._register(ws)
        except Refused:
            self._refused = True
            raise

    async def _command_lost(self):
        connection = self._cmd
        code = connection.close_code if connection is not None else None
        await super()._command_lost()  # Halt/interrupt before rediscovery.
        if getattr(self, '_discovering', False) and not self._refused and code not in (4001, 4003):
            raise _Rediscover()
