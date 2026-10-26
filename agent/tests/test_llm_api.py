"""Real optional SDK and CLI against a local fake OpenAI-compatible HTTP API.

No external LLM request, paid tokens or robot actions are involved.
"""
import asyncio
import gc
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from hei_agent import GraphPlanner, LLMConfig, LLMError, OpenAIChatModel
from hei_agent.catalog import MOCK_CATALOG


EXAMPLE = Path(__file__).resolve().parents[1] / 'src/hei_agent/examples/named_pose.json'


@pytest.fixture
def api(monkeypatch):
    pytest.importorskip('openai', reason='Install agent[llm,test] for SDK integration checks')
    for name in ('HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY', 'http_proxy', 'https_proxy', 'all_proxy'):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')
    state = SimpleNamespace(requests=[], responses=[])

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            state.requests.append({'path': self.path, 'body': json.loads(self.rfile.read(int(self.headers['Content-Length']))),
                                   'authorization': self.headers.get('Authorization')})
            answer = state.responses.pop(0) if state.responses else {'status': 500}
            time.sleep(answer.get('delay', 0))
            payload = answer.get('body', {
                'id': 'chatcmpl-local-test', 'object': 'chat.completion', 'created': 0,
                'model': 'local-test-model', 'choices': [{
                    'index': 0, 'finish_reason': answer.get('finish_reason', 'stop'),
                    'message': {'role': 'assistant', 'content': answer.get('content', EXAMPLE.read_text()),
                                'refusal': answer.get('refusal')},
                }], 'usage': {'prompt_tokens': 10, 'completion_tokens': 20, 'total_tokens': 30},
            })
            raw = json.dumps(payload).encode()
            try:
                self.send_response(answer.get('status', 200))
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError):
                pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=lambda: server.serve_forever(poll_interval=0.01), daemon=True)
    thread.start()
    state.url = f'http://127.0.0.1:{server.server_port}/v1'
    yield state
    server.shutdown()
    server.server_close()
    thread.join(timeout=2)


@pytest.mark.parametrize('mode', ['json', 'text'])
def test_actual_sdk_posts_expected_api_shape_and_saves_no_credentials(api, tmp_path, mode):
    api.responses.append({})
    key = 'fake-api-key-not-in-artifacts'
    model = OpenAIChatModel(LLMConfig('requested-model', api.url, key, response_format=mode))
    plan = asyncio.run(GraphPlanner(model, MOCK_CATALOG).generate('移动右臂', {'side': 'right'}, directory=tmp_path))
    assert plan.model == 'local-test-model'
    request = api.requests[0]
    assert request['path'] == '/v1/chat/completions'
    assert request['authorization'] == f'Bearer {key}'
    assert request['body']['model'] == 'requested-model'
    assert request['body']['stream'] is False
    assert ('response_format' in request['body']) == (mode == 'json')
    for artifact in plan.directory.iterdir():
        assert key not in artifact.read_text()
    assert json.loads((plan.directory / 'attempt-1.json').read_text())['usage']['total_tokens'] == 30


@pytest.mark.parametrize('answer,match', [
    ({'status': 401, 'body': {'error': {'message': 'echo fake-secret', 'type': 'auth'}}}, 'HTTP 401'),
    ({'status': 500, 'body': {'error': {'message': 'echo fake-secret', 'type': 'server'}}}, 'HTTP 500'),
    ({'finish_reason': 'length'}, 'incomplete'),
    ({'refusal': 'No'}, 'refused'),
    ({'body': {'choices': []}}, 'no choices'),
])
def test_api_failure_refusal_and_truncation_do_not_retry_or_execute(api, tmp_path, answer, match):
    api.responses.append(answer)
    model = OpenAIChatModel(LLMConfig('requested-model', api.url, 'fake-secret'))
    with pytest.raises(LLMError, match=match) as error:
        asyncio.run(GraphPlanner(model, MOCK_CATALOG).generate('任务', directory=tmp_path))
    assert 'fake-secret' not in str(error.value)
    assert len(api.requests) == 1
    assert not list(tmp_path.rglob('graph.json'))
    assert not list(tmp_path.rglob('*.jsonl'))
    for artifact in tmp_path.rglob('*.json'):
        assert 'fake-secret' not in artifact.read_text()


def test_api_overall_timeout_is_bounded_and_not_retried(api, tmp_path):
    api.responses.append({'delay': 0.5})
    model = OpenAIChatModel(LLMConfig('requested-model', api.url, timeout_sec=0.1))
    started = time.monotonic()
    with pytest.raises(LLMError, match='timed out'):
        asyncio.run(GraphPlanner(model, MOCK_CATALOG).generate('任务', directory=tmp_path))
    assert time.monotonic() - started < 1
    assert len(api.requests) <= 1


@pytest.mark.parametrize('setting,value', [
    ('ALL_PROXY', 'unsupported://fake-user:proxy-secret@127.0.0.1:9'),
    ('SSL_CERT_FILE', '/nonexistent/fake-secret-ca.pem'),
])
def test_transport_initialization_errors_are_not_responses_or_orphan_tasks(api, tmp_path, monkeypatch, setting, value):
    monkeypatch.setenv(setting, value)

    async def exercise():
        errors = []
        loop = asyncio.get_running_loop()
        old_handler = loop.get_exception_handler()
        loop.set_exception_handler(lambda loop, context: errors.append(context))
        try:
            model = OpenAIChatModel(LLMConfig('test', api.url, 'fake-secret'))
            with pytest.raises(LLMError, match='HTTP client initialization failed') as error:
                await GraphPlanner(model, MOCK_CATALOG).generate('任务', directory=tmp_path)
            assert 'No request was sent' in str(error.value)
            assert 'fake-secret' not in str(error.value) and 'proxy-secret' not in str(error.value)
            gc.collect()
            await asyncio.sleep(.05)
            assert not errors, errors
        finally:
            loop.set_exception_handler(old_handler)

    asyncio.run(exercise())
    assert not api.requests
    assert not list(tmp_path.rglob('graph.json'))
    for artifact in tmp_path.rglob('*.json'):
        assert 'proxy-secret' not in artifact.read_text()
        assert 'fake-secret' not in artifact.read_text()


def test_explicit_direct_mode_bypasses_bad_proxy_and_ca_environment(api, monkeypatch):
    monkeypatch.setenv('ALL_PROXY', 'socks://fake-user:proxy-secret@127.0.0.1:9')
    monkeypatch.setenv('SSL_CERT_FILE', '/nonexistent/ca.pem')
    monkeypatch.setenv('LLM_TRUST_ENV', 'false')
    monkeypatch.setenv('LLM_BASE_URL', api.url)
    monkeypatch.setenv('LLM_MODEL', 'test')
    api.responses.append({})
    reply = asyncio.run(OpenAIChatModel(LLMConfig.from_env()).complete([]))
    assert reply.model == 'local-test-model'
    assert len(api.requests) == 1


def test_sdk_constructor_failure_closes_owned_http_transport(api, monkeypatch):
    import hei_agent.llm as llm
    import openai
    factory = llm._http_client
    transports = []

    def create_transport(*args, **kwargs):
        transport = factory(*args, **kwargs)
        transports.append(transport)
        return transport

    def fail_sdk(**kwargs):
        raise TypeError('fake-secret-sdk-configuration')

    monkeypatch.setattr(llm, '_http_client', create_transport)
    monkeypatch.setattr(openai, 'AsyncOpenAI', fail_sdk)
    with pytest.raises(LLMError, match='SDK initialization failed') as error:
        asyncio.run(OpenAIChatModel(LLMConfig('test', api.url)).complete([]))
    assert 'fake-secret' not in str(error.value)
    assert len(transports) == 1 and transports[0].is_closed
    assert not api.requests


@pytest.mark.parametrize('name', ['HTTP_PROXY', 'HTTPS_PROXY', 'ALL_PROXY',
                                 'http_proxy', 'https_proxy', 'all_proxy'])
def test_legacy_socks_environment_alias_preserves_no_proxy(api, monkeypatch, name):
    value = 'socks://fake-user:proxy-secret@127.0.0.1:9'
    monkeypatch.setenv(name, value)
    api.responses.append({})
    reply = asyncio.run(OpenAIChatModel(LLMConfig('test', api.url)).complete([]))
    assert reply.model == 'local-test-model'
    assert len(api.requests) == 1  # NO_PROXY keeps the local API direct.
    assert os.environ[name] == value  # No global environment mutation.


def test_legacy_alias_keeps_proxy_route_for_non_bypassed_host(api, monkeypatch):
    import openai
    from hei_agent.llm import _http_client
    import httpx
    monkeypatch.setenv('ALL_PROXY', 'socks://127.0.0.1:9')

    async def exercise():
        async with _http_client(openai, timeout=1, trust_env=True) as transport:
            assert transport._transport_for_url(httpx.URL('https://example.com')) is not transport._transport
            assert transport._transport_for_url(httpx.URL(api.url)) is transport._transport

    asyncio.run(exercise())


def cli_env(api):
    env = dict(os.environ, PYTHONPATH=str(EXAMPLE.parents[2]),
               LLM_BASE_URL=api.url, LLM_MODEL='cli-model',
               LLM_API_KEY='fake-cli-key', NO_PROXY='127.0.0.1,localhost')
    env.pop('LD_PRELOAD', None)
    return env


@pytest.mark.parametrize('plan_only', [False, True])
def test_cli_natural_language_generation_and_optional_mock_execution(api, tmp_path, plan_only):
    api.responses.append({})
    command = [sys.executable, '-m', 'hei_agent', '--task', '右臂到预抓取位再到搬运位',
               '--trace-dir', str(tmp_path)]
    if plan_only:
        command.append('--plan-only')
    completed = subprocess.run(command, env=cli_env(api), capture_output=True, text=True, timeout=10)
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert Path(result['plan']['graph']).is_file()
    assert result['status'] == ('generated' if plan_only else 'succeeded')
    assert bool(list(tmp_path.glob('*.jsonl'))) == (not plan_only)
    assert bool(list(tmp_path.rglob('execution.json'))) == (not plan_only)


def test_cli_never_executes_graph_after_failed_generation(api, tmp_path):
    api.responses.append({'content': 'invalid graph'})
    command = [sys.executable, '-m', 'hei_agent', '--task', '任务', '--repair-attempts', '0',
               '--trace-dir', str(tmp_path)]
    completed = subprocess.run(command, env=cli_env(api), capture_output=True, text=True, timeout=10)
    assert completed.returncode == 2
    assert 'Graph generation failed' in completed.stderr
    assert 'fake-cli-key' not in completed.stderr
    assert not list(tmp_path.rglob('execution.json'))
    assert not list(tmp_path.rglob('*.jsonl'))
