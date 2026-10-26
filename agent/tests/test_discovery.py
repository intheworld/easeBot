"""Discovery metadata and real UDP loopback mDNS, no system daemon."""
import asyncio

import pytest

pytest.importorskip('zeroconf')
from zeroconf.asyncio import AsyncServiceInfo
from hei_discovery import (Endpoint, HubAdvertisement, HubDiscovery, SERVICE_TYPE,
                           endpoint_from_info, ipv4_interfaces)


def info(**changes):
    values = dict(port=4180, parsed_addresses=['127.0.0.1'], properties={
        'hub_id': 'lab', 'device_id': 'hei-01', 'protocol': 'mhs/v1', 'transport': 'ws'})
    values.update(changes)
    return AsyncServiceInfo(SERVICE_TYPE, 'test.' + SERVICE_TYPE, **values)


def test_parse_and_filter():
    assert endpoint_from_info(info()) == Endpoint('lab', ('ws://127.0.0.1:4180',))
    assert endpoint_from_info(info(), hub_id='other') is None
    assert endpoint_from_info(info(), device_id='other') is None


@pytest.mark.parametrize('changes', [dict(port=0), dict(parsed_addresses=[]),
    dict(parsed_addresses=['0.0.0.0']), dict(properties={}),
    dict(properties={'hub_id': 'lab', 'protocol': 'bad', 'transport': 'ws'}),
    dict(properties={'hub_id': 'lab', 'protocol': 'mhs/v1', 'transport': 'wss'}),
    dict(properties={'hub_id': '../bad', 'protocol': 'mhs/v1', 'transport': 'ws'})])
def test_bad_advertisements(changes):
    assert endpoint_from_info(info(**changes)) is None


@pytest.mark.parametrize('value', ['0.0.0.0', '224.0.0.251', '::1', 'eth0', '1.2.3.4,'])
def test_invalid_interfaces(value):
    with pytest.raises(ValueError):
        ipv4_interfaces(value)


def test_duplicate_identity_is_ambiguous():
    discovery = HubDiscovery(interfaces='127.0.0.1')
    endpoint = Endpoint('lab', ('ws://127.0.0.1:4180',))
    discovery.entries['first'] = endpoint
    assert discovery.candidate() == endpoint
    discovery.entries['second'] = endpoint
    assert discovery.candidate() is None
    with pytest.raises(ValueError):
        HubAdvertisement(4180, interfaces='127.0.0.1', bind_host='192.0.2.1')


def test_real_mdns_late_publish_removal_and_new_port():
    async def exercise():
        async with HubDiscovery(hub_id='lab', device_id='hei-01', interfaces='127.0.0.1') as discovery:
            waiter = asyncio.create_task(discovery.wait())
            await asyncio.sleep(.05)
            assert not waiter.done()
            async with HubAdvertisement(4180, hub_id='lab', interfaces='127.0.0.1'):
                assert (await asyncio.wait_for(waiter, 8)).urls == ('ws://127.0.0.1:4180',)
            deadline = asyncio.get_running_loop().time() + 5
            while discovery.entries:
                assert asyncio.get_running_loop().time() < deadline
                await asyncio.sleep(.05)
            async with HubAdvertisement(4181, hub_id='lab', interfaces='127.0.0.1'):
                assert (await asyncio.wait_for(discovery.wait(), 8)).urls == ('ws://127.0.0.1:4181',)
    asyncio.run(exercise())
