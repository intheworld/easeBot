"""Run local mock or explicitly selected, discovered MHS skills."""
import argparse
import asyncio
from contextlib import AsyncExitStack
from dataclasses import replace
from importlib.resources import files
import json
import os
import sys

from .agent import Agent
from .executor import RunStatus
from .graph import Graph
from .llm import LLMConfig, OpenAIChatModel
from .mock import MockConnector
from .node import NodeRegistry
from .planner import GraphPlanner, validate_plan


async def run(args):
    if args.backend in ('mhs', 'embedded'):
        from .mhs import HubClient, MHSConnector
        if not (args.task or args.graph or args.list_skills or args.serve):
            raise ValueError('MHS backend requires --task, --graph or --list-skills; no implicit demo motion')
        if args.backend == 'embedded':
            from .hub import EmbeddedHub
            transport = EmbeddedHub(args.listen_host, args.listen_port, device_id=args.device_id)
        else:
            transport = HubClient(args.hub_url)
        async with AsyncExitStack() as stack:
            hub = await stack.enter_async_context(transport)
            if args.backend == 'embedded':
                print(f'MHS listening on {hub.host}:{hub.port}; waiting for {args.device_id}', file=sys.stderr)
                if not args.no_discovery:
                    try:
                        from hei_discovery import HubAdvertisement
                    except ImportError as exc:
                        raise RuntimeError('Install packages/hei-discovery and agent[mhs] first') from exc
                    await stack.enter_async_context(HubAdvertisement(hub.port, hub_id=args.hub_id,
                        device_id=args.device_id, interfaces=args.discovery_interface, bind_host=hub.host))
                    print(f'MHS advertising hub_id={args.hub_id}', file=sys.stderr)
                if args.serve:
                    return await serve_requests(args, hub)
                await hub.wait_device(args.device_id, args.wait_device)
            connector = MHSConnector(hub, args.device_id, artifact_dir=f'{args.trace_dir}/observations')
            await connector.discover()
            return await run_policy(args, connector)
    return await run_policy(args, MockConnector(ready=not args.not_ready, delay_sec=args.delay_sec, fail_at=args.fail_at))


async def serve_requests(args, hub):
    """Serial JSON-lines stdin interface; Hub stays up between requests."""
    from .mhs import MHSConnector
    reader = asyncio.StreamReader(limit=1024 * 1024)
    protocol = asyncio.StreamReaderProtocol(reader)
    loop = asyncio.get_running_loop()
    transport, _ = await loop.connect_read_pipe(lambda: protocol, sys.stdin)
    print('MHS ready; submit JSON lines on stdin (EOF exits)', file=sys.stderr)
    try:
        while True:
            line = await reader.readline()
            if not line:
                return {'status': 'stopped'}, 0
            try:
                value = json.loads(line)
                allowed = {'task', 'graph', 'list_skills', 'side', 'plan_only', 'validate'}
                if type(value) is not dict or value.keys() - allowed:
                    raise ValueError('Request must be an object containing task/graph/list_skills and optional side/plan_only/validate')
                if sum(bool(value.get(key)) for key in ('task', 'graph', 'list_skills')) != 1:
                    raise ValueError('Specify exactly one of task, graph or list_skills')
                for key in ('task', 'graph'):
                    if key in value and (type(value[key]) is not str or not value[key].strip()):
                        raise ValueError(f'{key} must be a nonempty string')
                for key in ('list_skills', 'plan_only', 'validate'):
                    if key in value and type(value[key]) is not bool:
                        raise ValueError(f'{key} must be boolean')
                request = argparse.Namespace(**vars(args))
                request.task, request.graph, request.list_skills = None, None, False
                request.plan_only = request.validate = False
                vars(request).update(value)
                if request.side not in ('left', 'right'):
                    raise ValueError('side must be left or right')
                if request.plan_only and not request.task or request.validate and not request.graph:
                    raise ValueError('plan_only requires task; validate requires graph')
                await hub.wait_device(args.device_id, args.wait_device)
                connector = MHSConnector(hub, args.device_id, artifact_dir=f'{args.trace_dir}/observations')
                await connector.discover()
                output, _ = await run_policy(request, connector)
                print(json.dumps(output, ensure_ascii=False), flush=True)
                if output.get('cleanup_complete') is False:
                    return {'status': 'blocked', 'message': 'Cleanup unconfirmed; stop accepting tasks'}, 1
            except (ValueError, OSError, RuntimeError, asyncio.TimeoutError) as exc:
                print(json.dumps({'status': 'error', 'message': str(exc)}, ensure_ascii=False), flush=True)
    finally:
        transport.close()


async def run_policy(args, connector):
    if args.list_skills:
        return {'backend': args.backend, 'revision': connector.catalog.revision,
                'execution_backend': connector.catalog.backend,
                'skills': connector.catalog.describe()}, 0
    plan = None
    plan_info = None
    if args.task:
        config = LLMConfig.from_env(model=args.llm_model, base_url=args.llm_base_url,
                                    timeout_sec=args.llm_timeout, response_format=args.llm_response_format)
        planner = GraphPlanner(OpenAIChatModel(config), connector.catalog, repair_attempts=args.repair_attempts)
        plan = await planner.generate(args.task, {'side': args.side},
                                      directory=args.plan_dir or f'{args.trace_dir}/plans')
        graph = plan.graph
        plan_info = {'id': plan.id, 'graph': str(plan.graph_path), 'model': plan.model,
                     'attempts': plan.attempts, 'directory': str(plan.directory)}
        if args.plan_only:
            return {'status': 'generated', 'executed': False, 'plan': plan_info}, 0
    elif args.graph:
        graph = Graph.load(args.graph)
    else:
        graph = Graph.from_dict(json.loads(files('hei_agent').joinpath('examples/named_pose.json').read_text()))
    unknown = {node.kind for node in graph.nodes} - NodeRegistry().snapshot().keys()
    if unknown:
        raise ValueError(f'Unregistered node kinds: {sorted(unknown)}')
    if args.backend in ('mhs', 'embedded'):
        # An unbound file is a template: validate its entire contract before
        # pinning it to the catalog just discovered for this request. A graph
        # already pinned to an older revision must still fail, never rebind.
        validate_plan(graph, {'side': args.side}, connector.catalog)
        if graph.catalog_revision is None:
            if connector.catalog.revision is None:
                raise ValueError('MHS catalog has no discovered revision')
            graph = replace(graph, catalog_revision=connector.catalog.revision)
    if args.validate:
        return {'graph': graph.name, 'valid': True, 'nodes': len(graph.nodes)}, 0
    result = await Agent(graph, connector, cancel_timeout_sec=12 if args.backend in ('mhs', 'embedded') else 5).run(
        {'side': args.side}, trace_dir=args.trace_dir)
    output = {'backend': args.backend, 'run_id': result.run_id, 'status': result.status.value,
              'execution_backend': connector.catalog.backend,
              'steps': result.steps, 'outputs': result.outputs, 'message': result.message,
              'cleanup_complete': result.cleanup_complete, 'trace': result.trace_path}
    if plan is not None:
        output['plan'] = plan_info
        with (plan.directory / 'execution.json').open('x', encoding='utf-8') as stream:
            json.dump({'run_id': result.run_id, 'status': result.status.value, 'trace': result.trace_path}, stream)
    return output, 0 if result.status is RunStatus.SUCCEEDED else 1


def main():
    parser = argparse.ArgumentParser(description='Generate/execute a policy using local mock or discovered MHS skills')
    parser.add_argument('--backend', choices=['mock', 'mhs', 'embedded'], default='mock')
    parser.add_argument('--listen-host', default='0.0.0.0', help='Embedded MHS listen address (default all interfaces)')
    parser.add_argument('--listen-port', type=int, default=4180, help='Embedded MHS port (default 4180)')
    parser.add_argument('--hub-id', default='hei-agent', help='Stable discovery identity (embedded Hub)')
    parser.add_argument('--discovery-interface', default='', help='Comma separated local IPv4 addresses; empty means all usable NICs')
    parser.add_argument('--no-discovery', action='store_true', help='Disable embedded Hub mDNS advertisement')
    parser.add_argument('--serve', action='store_true', help='Embedded Hub stays up; accept serial JSON requests on stdin until EOF')
    parser.add_argument('--wait-device', type=float, default=30, help='Seconds to wait for the embedded robot connection')
    parser.add_argument('--hub-url', default=os.getenv('MHS_HUB_URL', 'ws://127.0.0.1:4180'))
    parser.add_argument('--device-id', default=os.getenv('MHS_DEVICE_ID', 'hei-01'))
    parser.add_argument('--list-skills', action='store_true', help='Discover and print skills without executing a graph')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--graph', help='HEI graph/v1 JSON; defaults to the packaged named-pose demo')
    source.add_argument('--task', help='Generate a graph from a natural language task, then execute with selected backend')
    parser.add_argument('--plan-only', action='store_true', help='Generate and save a graph without executing it; requires --task')
    parser.add_argument('--plan-dir', help='Plan artifact root; defaults to TRACE_DIR/plans')
    parser.add_argument('--llm-model', help='Model ID; defaults to LLM_MODEL')
    parser.add_argument('--llm-base-url', help='API base URL; defaults to LLM_BASE_URL')
    parser.add_argument('--llm-timeout', type=float, help='Overall API request timeout in seconds')
    parser.add_argument('--llm-response-format', choices=['json', 'text'], help='Override LLM_RESPONSE_FORMAT')
    parser.add_argument('--repair-attempts', type=int, choices=[0, 1, 2], default=1,
                        help='Extra model calls to repair invalid graphs, not motion retries (default 1)')
    parser.add_argument('--side', choices=['left', 'right'], default='right')
    parser.add_argument('--not-ready', action='store_true', help='Exercise the readiness failure branch')
    parser.add_argument('--fail-at', choices=['pre_grasp', 'transport'])
    parser.add_argument('--delay-sec', type=float, default=0.01)
    parser.add_argument('--trace-dir', default='log/hei_agent')
    parser.add_argument('--validate', action='store_true', help='Validate structure and registered node kinds only')
    args = parser.parse_args()
    if args.serve and (args.backend != 'embedded' or args.task or args.graph or args.list_skills or args.plan_only or args.validate):
        parser.error('--serve requires --backend embedded and receives task requests through stdin')
    if args.plan_only and not args.task:
        parser.error('--plan-only requires --task')
    if args.task is not None and not args.task.strip():
        parser.error('--task must be nonempty')
    if args.task and args.validate:
        parser.error('--validate applies to existing graphs; use --plan-only with --task')
    try:
        output, exit_code = asyncio.run(run(args))
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return exit_code
    except KeyboardInterrupt:
        return 130
    except (ValueError, OSError, RuntimeError, asyncio.TimeoutError) as exc:
        print(f'hei-agent: {exc}', file=sys.stderr)
        return 2
