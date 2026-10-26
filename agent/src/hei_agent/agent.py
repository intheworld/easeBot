"""An agent owns an executable policy; GraphPlanner handles LLM generation separately."""
from .executor import Executor
from .graph import Graph


class Agent:
    def __init__(self, graph: Graph, connector=None, registry=None, *, cancel_timeout_sec=5.0):
        self.graph = graph
        self.executor = Executor(connector, registry, cancel_timeout_sec=cancel_timeout_sec)

    async def run(self, inputs=None, *, trace_dir=None):
        return await self.executor.run(self.graph, inputs, trace_dir=trace_dir)

    def cancel(self):
        return self.executor.cancel()
