"""Optional OpenAI-compatible Chat Completions adapter; no execution tools."""
import asyncio
from dataclasses import dataclass, field
import os
from typing import Protocol
from urllib.parse import urlsplit

from .graph import positive_seconds


class LLMError(ValueError):
    pass


@dataclass(frozen=True)
class LLMConfig:
    model: str
    base_url: str = 'https://api.openai.com/v1'
    api_key: str = field(default='', repr=False)
    timeout_sec: float = 60.0
    response_format: str = 'json'
    trust_env: bool = True

    def __post_init__(self):
        if not isinstance(self.model, str) or not self.model.strip():
            raise LLMError('Set LLM_MODEL or --llm-model to your provider model ID')
        parsed = urlsplit(self.base_url)
        if (parsed.scheme not in ('http', 'https') or not parsed.hostname or
                parsed.username or parsed.password or parsed.query or parsed.fragment):
            raise LLMError('LLM base URL must be an http(s) API base without credentials, query or fragment')
        if parsed.hostname == 'api.openai.com' and not self.api_key:
            raise LLMError('Set LLM_API_KEY or OPENAI_API_KEY')
        if not isinstance(self.api_key, str) or any(char in self.api_key for char in '\r\n'):
            raise LLMError('Invalid API key format')
        positive_seconds(self.timeout_sec, 'LLM timeout_sec')
        if self.response_format not in ('json', 'text'):
            raise LLMError('LLM response format must be json or text')
        if type(self.trust_env) is not bool:
            raise LLMError('LLM trust_env must be boolean')

    @classmethod
    def from_env(cls, *, model=None, base_url=None, timeout_sec=None, response_format=None):
        try:
            timeout = timeout_sec if timeout_sec is not None else float(os.getenv('LLM_TIMEOUT_SEC', '60'))
        except ValueError as exc:
            raise LLMError('LLM_TIMEOUT_SEC must be a number') from exc
        trust_env = os.getenv('LLM_TRUST_ENV', 'true').lower().strip()
        if trust_env not in ('true', 'false'):
            raise LLMError('LLM_TRUST_ENV must be true or false')
        return cls(
            model=model if model is not None else os.getenv('LLM_MODEL', ''),
            base_url=base_url if base_url is not None else os.getenv('LLM_BASE_URL', 'https://api.openai.com/v1'),
            api_key=os.getenv('LLM_API_KEY') or os.getenv('OPENAI_API_KEY', ''),
            timeout_sec=timeout,
            response_format=response_format if response_format is not None else os.getenv('LLM_RESPONSE_FORMAT', 'json'),
            trust_env=trust_env == 'true',
        )


@dataclass(frozen=True)
class ModelReply:
    text: str
    model: str
    usage: dict = field(default_factory=dict)


class ChatModel(Protocol):
    async def complete(self, messages: list[dict]) -> ModelReply:
        ...


def _http_client(openai, *, timeout, trust_env):
    """Keep HTTPX's proxy precedence/NO_PROXY/CA handling, accept socks://.

    HTTPX 0.28 has no public callback for transforming environment proxy URLs.
    Isolate its proxy-map hook here (dependency bounded to 0.28.x), rather than
    changing os.environ or disabling the user's proxies for the whole process.
    """
    import httpx
    from httpx._utils import get_environment_proxies

    class CompatibleHttpClient(openai.DefaultAsyncHttpxClient):
        def _get_proxy_map(self, proxy, allow_env_proxies):
            if proxy is None and allow_env_proxies:
                result = {}
                for pattern, url in get_environment_proxies().items():
                    if url is not None and url.lower().startswith('socks://'):
                        url = 'socks5://' + url[len('socks://'):]
                    result[pattern] = None if url is None else httpx.Proxy(url=url)
                return result
            return super()._get_proxy_map(proxy, allow_env_proxies)

    return CompatibleHttpClient(timeout=timeout, trust_env=trust_env)


class OpenAIChatModel:
    def __init__(self, config: LLMConfig):
        self.config = config

    async def complete(self, messages):
        try:
            import openai
        except ImportError as exc:
            raise LLMError('Install the LLM extra: pip install -e "./agent[llm]"') from exc

        async def request():
            # Construct the transport separately. The SDK's default wrapper
            # schedules cleanup in __del__, even when HTTPX initialization has
            # failed before _transport exists (e.g. invalid proxy environment).
            try:
                transport = _http_client(openai,
                    timeout=self.config.timeout_sec, trust_env=self.config.trust_env)
            except ImportError as exc:
                raise LLMError('LLM transport dependencies missing; install agent[llm] including SOCKS support') from exc
            except (ValueError, TypeError, OSError) as exc:
                raise LLMError(
                    f'LLM HTTP client initialization failed ({type(exc).__name__}); '
                    'check HTTP_PROXY/HTTPS_PROXY/ALL_PROXY (including lowercase variants) '
                    'and SSL_CERT_FILE/SSL_CERT_DIR. Supported proxy schemes: http, https, socks5, socks5h '
                    '(legacy socks:// is normalized); '
                    'for direct connections set LLM_TRUST_ENV=false '
                    '(also disables environment CA settings). No request was sent') from exc
            # Custom endpoints without authentication still need a placeholder
            # for the SDK. No key is ever added to prompts or plan artifacts.
            async with transport:
                try:
                    client = openai.AsyncOpenAI(
                        api_key=self.config.api_key or 'not-required',
                        base_url=self.config.base_url.rstrip('/') + '/',
                        timeout=self.config.timeout_sec, max_retries=0, http_client=transport)
                except (ValueError, TypeError, AttributeError) as exc:
                    raise LLMError(f'LLM SDK initialization failed ({type(exc).__name__}); '
                                   'check client configuration and SDK dependencies. No request was sent') from exc
                options = {'response_format': {'type': 'json_object'}} if self.config.response_format == 'json' else {}
                completion = await client.chat.completions.create(
                    model=self.config.model, messages=messages, stream=False, **options)
                if not completion.choices:
                    raise LLMError('LLM returned no choices')
                choice = completion.choices[0]
                if getattr(choice.message, 'refusal', None):
                    raise LLMError('LLM refused the planning request')
                if choice.finish_reason != 'stop':
                    raise LLMError('LLM response is incomplete or requested unsupported tool calls')
                if not isinstance(choice.message.content, str) or not choice.message.content.strip():
                    raise LLMError('LLM returned no graph text')
                if len(choice.message.content) > 256_000:
                    raise LLMError('LLM graph response exceeds 256000 characters')
                usage = completion.usage.model_dump(exclude_none=True) if completion.usage else {}
                return ModelReply(choice.message.content, completion.model or self.config.model, usage)
        try:
            return await asyncio.wait_for(request(), timeout=self.config.timeout_sec)
        except (asyncio.TimeoutError, openai.APITimeoutError) as exc:
            raise LLMError('LLM request timed out; no graph was executed') from exc
        except openai.APIStatusError as exc:
            # Provider error bodies/URLs may echo credentials; report only status.
            raise LLMError(f'LLM API returned HTTP {exc.status_code}; check endpoint/model/credentials') from exc
        except openai.APIConnectionError as exc:
            raise LLMError('Cannot connect to the configured LLM API') from exc
        except openai.APIError as exc:
            raise LLMError('LLM API returned an invalid response') from exc
        except ImportError as exc:
            raise LLMError('LLM transport dependencies missing; install agent[llm] including SOCKS support') from exc
        except LLMError:
            raise
        except (AttributeError, KeyError, TypeError, ValueError) as exc:
            raise LLMError('LLM API returned a malformed response') from exc
