"""Project-specific DNS-SD convention, independent of ROS and the MHS SDK."""
import asyncio
from dataclasses import dataclass
import ipaddress
import logging
import re
import uuid

import ifaddr
from zeroconf import IPVersion, ServiceStateChange
from zeroconf.asyncio import AsyncServiceBrowser, AsyncServiceInfo, AsyncZeroconf

SERVICE_TYPE = '_hei-mhs._tcp.local.'
log = logging.getLogger(__name__)


def validate_hub_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-z0-9][a-z0-9-]{0,31}', value):
        raise ValueError('hub_id must contain 1..32 lowercase letters, digits or hyphens')
    return value


def ipv4_interfaces(value=''):
    """Comma separated local IPv4 addresses; empty means all usable IPv4 NICs."""
    if value:
        addresses = list(dict.fromkeys(part.strip() for part in value.split(',')))
    else:
        addresses = list(dict.fromkeys(address.ip for adapter in ifaddr.get_adapters()
                         for address in adapter.ips if isinstance(address.ip, str)
                         and not ipaddress.ip_address(address.ip).is_loopback))
    for address in addresses:
        ip = ipaddress.IPv4Address(address)
        if ip.is_unspecified or ip.is_multicast:
            raise ValueError('Discovery interfaces must be local unicast IPv4 addresses')
    if not addresses:
        raise ValueError('No usable IPv4 discovery interface; specify its local IPv4 address')
    return addresses


@dataclass(frozen=True)
class Endpoint:
    hub_id: str
    urls: tuple


def endpoint_from_info(info, *, hub_id='', device_id=''):
    properties = info.properties
    if properties.get(b'protocol') != b'mhs/v1' or properties.get(b'transport') != b'ws':
        return None
    try:
        identity = validate_hub_id(properties[b'hub_id'].decode('ascii'))
        if hub_id and identity != hub_id:
            return None
        if device_id and properties.get(b'device_id') != device_id.encode('ascii'):
            return None
        addresses = sorted(set(info.parsed_addresses(IPVersion.V4Only)))
        addresses = [address for address in addresses if not (
            ipaddress.IPv4Address(address).is_unspecified or ipaddress.IPv4Address(address).is_multicast)]
        if not addresses or not 1 <= info.port <= 65535:
            return None
        return Endpoint(identity, tuple(f'ws://{address}:{info.port}' for address in addresses))
    except (KeyError, ValueError, UnicodeError, TypeError):
        return None


class HubAdvertisement:
    def __init__(self, port, *, hub_id='hei-agent', device_id='hei-01', interfaces='', bind_host='0.0.0.0'):
        self.hub_id = validate_hub_id(hub_id)
        self.addresses = ipv4_interfaces(interfaces)
        if bind_host != '0.0.0.0':
            if bind_host not in self.addresses:
                raise ValueError('Hub bind address must be a discovery interface, or use --no-discovery')
            self.addresses = [bind_host]
        if not 1 <= port <= 65535:
            raise ValueError('Advertisement requires a bound TCP port')
        unique = uuid.uuid4().hex[:12]
        self.info = AsyncServiceInfo(SERVICE_TYPE, f'{hub_id}-{unique}.{SERVICE_TYPE}',
            port=port, parsed_addresses=self.addresses, server=f'hei-{unique}.local.',
            properties={'hub_id': hub_id, 'device_id': device_id, 'protocol': 'mhs/v1', 'transport': 'ws'},
            host_ttl=15, other_ttl=15)
        self.zc = None

    async def __aenter__(self):
        self.zc = AsyncZeroconf(interfaces=self.addresses, ip_version=IPVersion.V4Only)
        try:
            broadcast = await self.zc.async_register_service(self.info)
            await broadcast
        except BaseException:
            await self.zc.async_close()
            raise
        return self

    async def __aexit__(self, *args):
        try:
            broadcast = await self.zc.async_unregister_service(self.info)
            await broadcast
        finally:
            await self.zc.async_close()


class HubDiscovery:
    def __init__(self, *, hub_id='', device_id='', interfaces='', settle_sec=1.0):
        if hub_id:
            validate_hub_id(hub_id)
        self.hub_id, self.device_id = hub_id, device_id
        self.interfaces = ipv4_interfaces(interfaces)
        self.settle_sec = settle_sec
        self.entries, self.tasks = {}, {}
        self.changed = asyncio.Event()
        self.zc = self.browser = None
        self.ready_at = 0

    async def __aenter__(self):
        self.zc = AsyncZeroconf(interfaces=self.interfaces, ip_version=IPVersion.V4Only)
        try:
            self.browser = AsyncServiceBrowser(self.zc.zeroconf, SERVICE_TYPE, handlers=[self._event])
            self.ready_at = asyncio.get_running_loop().time() + self.settle_sec
        except BaseException:
            await self.zc.async_close()
            raise
        return self

    async def __aexit__(self, *args):
        await self.browser.async_cancel()
        for task in self.tasks.values():
            task.cancel()
        await asyncio.gather(*self.tasks.values(), return_exceptions=True)
        await self.zc.async_close()

    def _event(self, zeroconf, service_type, name, state_change):
        previous = self.tasks.pop(name, None)
        if previous is not None:
            previous.cancel()
        if state_change is ServiceStateChange.Removed:
            self.entries.pop(name, None)
            self.changed.set()
        elif len(self.tasks) < 64:
            self.tasks[name] = asyncio.create_task(self._resolve(name))

    async def _resolve(self, name):
        try:
            info = AsyncServiceInfo(SERVICE_TYPE, name)
            resolved = await info.async_request(self.zc.zeroconf, 2000)
            value = endpoint_from_info(info, hub_id=self.hub_id, device_id=self.device_id) if resolved else None
            self.entries.pop(name, None)
            if value is not None and len(self.entries) < 64:
                self.entries[name] = value
            self.changed.set()
        except Exception:
            log.exception('Could not resolve MHS service %s', name)
        finally:
            if self.tasks.get(name) is asyncio.current_task():
                self.tasks.pop(name, None)

    def candidate(self):
        # Count service instances, not just hub_id: duplicate identities are ambiguous.
        return next(iter(self.entries.values())) if len(self.entries) == 1 else None

    async def wait(self):
        remaining = self.ready_at - asyncio.get_running_loop().time()
        if remaining > 0:
            await asyncio.sleep(remaining)
        while True:
            self.changed.clear()
            candidate = self.candidate()
            if candidate is not None:
                return candidate
            if len(self.entries) > 1:
                log.warning('Multiple matching MHS Hubs; waiting for an unambiguous hub_id')
            await self.changed.wait()
