"""Node implementations and the transport-independent skill boundary."""
from abc import ABC, abstractmethod
import asyncio
from dataclasses import dataclass, field
import inspect
from typing import Awaitable, Callable, Protocol

from .graph import GraphError, json_data


@dataclass(frozen=True)
class NodeResult:
    outcome: str = 'success'
    outputs: dict = field(default_factory=dict)
    message: str = ''

    def validate(self):
        if not isinstance(self.outcome, str) or not self.outcome:
            raise GraphError('Node outcome must be a nonempty string')
        if type(self.outputs) is not dict or not isinstance(self.message, str):
            raise GraphError('Node outputs must be an object and message must be a string')
        json_data(self.outputs)
        return self


class SkillConnector(Protocol):
    async def call(self, skill: str, arguments: dict, *, call_id: str) -> NodeResult:
        """Return only after the skill has reached a terminal state."""
        ...

    async def cancel(self, call_id: str) -> None:
        """Confirm termination of this invocation; raise if termination is uncertain.

        This must also handle cancellation racing with dispatch/acceptance. It is
        not sufficient to acknowledge that a cancel request has merely been sent.
        """
        ...


@dataclass(frozen=True)
class NodeContext:
    run_id: str
    node_id: str
    step: int
    visit: int
    connector: SkillConnector | None
    cancel_requested: asyncio.Event

    @property
    def call_id(self):
        return f'{self.run_id}:{self.node_id}:{self.visit}'


class BaseNode(ABC):
    @abstractmethod
    async def execute(self, context: NodeContext, inputs: dict) -> NodeResult:
        """Cooperative async code; do not block the event loop."""
        raise NotImplementedError

    async def cancel(self, context: NodeContext) -> None:
        """Extra cleanup after execute is cancelled. Local functions use finally."""


class FunctionNode(BaseNode):
    def __init__(self, function: Callable[[NodeContext, dict], Awaitable[NodeResult]]):
        if not inspect.iscoroutinefunction(function):
            raise TypeError('FunctionNode requires an async function')
        self.function = function

    async def execute(self, context, inputs):
        return await self.function(context, inputs)


class SkillNode(BaseNode):
    async def execute(self, context, inputs):
        if set(inputs) != {'skill', 'arguments'}:
            raise ValueError('skill node requires exactly skill and arguments')
        if not isinstance(inputs['skill'], str) or not inputs['skill'] or type(inputs['arguments']) is not dict:
            raise ValueError('skill must be a name and arguments must be an object')
        if context.connector is None:
            raise RuntimeError('No skill connector configured')
        return await context.connector.call(inputs['skill'], inputs['arguments'], call_id=context.call_id)

    async def cancel(self, context):
        if context.connector is not None:
            await context.connector.cancel(context.call_id)


class BranchNode(BaseNode):
    async def execute(self, context, inputs):
        if set(inputs) != {'condition'} or type(inputs['condition']) is not bool:
            raise ValueError('branch condition must be a boolean')
        return NodeResult('true' if inputs['condition'] else 'false')


class EndNode(BaseNode):
    async def execute(self, context, inputs):
        if set(inputs) - {'status', 'outputs', 'message'}:
            raise ValueError('Unknown end node input')
        if inputs['status'] not in ('succeeded', 'failed'):
            raise ValueError('Invalid end status')
        return NodeResult('success' if inputs['status'] == 'succeeded' else 'failure',
                          inputs.get('outputs', {}), inputs.get('message', ''))


class NodeRegistry:
    def __init__(self):
        self._nodes = {'skill': SkillNode(), 'branch': BranchNode(), 'end': EndNode()}

    def register(self, kind: str, implementation: BaseNode):
        if not isinstance(kind, str) or not kind.strip():
            raise ValueError('Node kind must be nonempty')
        if kind in self._nodes:
            raise ValueError(f'Node kind already registered: {kind}')
        if not isinstance(implementation, BaseNode):
            raise TypeError('Expected a BaseNode implementation')
        self._nodes[kind] = implementation
        return self

    def snapshot(self):
        return dict(self._nodes)
