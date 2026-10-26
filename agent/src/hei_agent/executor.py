"""Sequential, outcome-routed execution with bounded loops and cancellation."""
import asyncio
from dataclasses import dataclass
from enum import Enum
import time
from uuid import uuid4

from .graph import Graph, GraphError, json_data, positive_seconds, resolve
from .node import NodeContext, NodeRegistry, NodeResult
from .trace import Trace


class RunStatus(str, Enum):
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    CANCELLED = 'cancelled'
    TIMED_OUT = 'timed_out'


@dataclass(frozen=True)
class ExecutionResult:
    run_id: str
    status: RunStatus
    steps: int
    outputs: dict
    node_outputs: dict
    message: str
    cleanup_complete: bool
    events: tuple[dict, ...]
    trace_path: str | None


class Executor:
    """One active run per instance. Use from a single asyncio event loop.

    Independent executors are not a global robot resource arbiter. A future real
    connector must enforce ownership across all motion clients.
    """
    def __init__(self, connector=None, registry=None, *, cancel_timeout_sec=5.0):
        positive_seconds(cancel_timeout_sec, 'cancel_timeout_sec')
        self.connector = connector
        self.registry = registry if registry is not None else NodeRegistry()
        self.cancel_timeout_sec = cancel_timeout_sec
        self._busy = False
        self._cancel = None
        self._pending = set()
        self._quarantined = False

    def cancel(self):
        """Request run cancellation; await run() to observe cleanup completion."""
        if self._cancel is None:
            return False
        self._cancel.set()
        return True

    def _track(self, task):
        self._pending.add(task)

        def finished(done):
            self._pending.discard(done)
            if not done.cancelled():
                done.exception()  # Consume late errors after a cleanup deadline.
        task.add_done_callback(finished)

    async def _stop(self, task, implementation, context):
        # A dispatch coroutine ending does not establish that a remote action ended.
        self._quarantined = True
        task.cancel()
        cleanup = asyncio.create_task(implementation.cancel(context))
        done, pending = await asyncio.wait({task, cleanup}, timeout=self.cancel_timeout_sec)
        problems = []
        for item in done:
            if item.cancelled():
                if item is cleanup:
                    problems.append('cancel hook was cancelled')
            elif item.exception() is not None:
                # The execution error triggered this cleanup; only hook errors
                # make the acknowledgement itself unsuccessful.
                if item is cleanup:
                    problems.append(f'cancel hook: {item.exception()}')
        for item in pending:
            self._track(item)
        if pending:
            problems.append('cleanup deadline exceeded')
            cleanup.cancel()
        complete = not problems
        self._quarantined = not complete
        return complete, '; '.join(problems)

    async def _execute_node(self, implementation, context, inputs, timeout):
        expires = time.monotonic() + timeout

        async def invoke():
            # A deadline can expire while the event loop or trace I/O is busy.
            # Do not dispatch a new external call just to cancel it afterwards.
            if context.cancel_requested.is_set():
                return NodeResult('cancelled', message='Cancelled before dispatch')
            if time.monotonic() >= expires:
                return NodeResult('timeout', message='Deadline expired before dispatch')
            result = await implementation.execute(context, inputs)
            if not isinstance(result, NodeResult):
                raise TypeError('Nodes must return NodeResult')
            result.validate()
            return NodeResult(result.outcome, json_data(result.outputs), result.message)

        task = asyncio.create_task(invoke())
        cancellation = asyncio.create_task(context.cancel_requested.wait())
        cleanup_task = None

        async def stop():
            nonlocal cleanup_task
            if cleanup_task is None:
                self._quarantined = True
                cleanup_task = asyncio.create_task(self._stop(task, implementation, context))
                self._track(cleanup_task)
            return await asyncio.shield(cleanup_task)

        try:
            done, _ = await asyncio.wait({task, cancellation}, timeout=timeout,
                                         return_when=asyncio.FIRST_COMPLETED)
            if context.cancel_requested.is_set():
                complete, detail = await stop()
                return NodeResult('cancelled', message=detail or 'Run cancellation requested'), complete
            if task in done:
                try:
                    result = task.result()
                except asyncio.CancelledError:
                    error = 'Node cancelled itself'
                except Exception as exc:
                    error = f'{type(exc).__name__}: {exc}'
                else:
                    if time.monotonic() < expires:
                        return result, True
                    complete, detail = await stop()
                    return NodeResult('timeout', message=detail or 'Node execution deadline exceeded'), complete
                complete, detail = await stop()
                return NodeResult('error', message=error + (f'; {detail}' if detail else '')), complete
            complete, detail = await stop()
            return NodeResult('timeout', message=detail or 'Node execution deadline exceeded'), complete
        except asyncio.CancelledError:
            # Keep standard asyncio Task.cancel semantics, but clean up first.
            context.cancel_requested.set()
            await stop()
            raise
        finally:
            cancellation.cancel()
            await asyncio.gather(cancellation, return_exceptions=True)

    async def run(self, graph: Graph, inputs=None, *, trace_dir=None):
        if self._busy or self._pending or self._quarantined:
            raise RuntimeError('Executor busy or previous cleanup unconfirmed; reconcile before reuse')
        # Snapshot mutable user data and validate every kind before the first action.
        graph = Graph.from_dict(graph.to_dict())
        actual_revision = getattr(getattr(self.connector, 'catalog', None), 'revision', None)
        if graph.catalog_revision is not None and graph.catalog_revision != actual_revision:
            raise GraphError('Graph skill catalog revision differs from the connector; replan before executing')
        implementations = self.registry.snapshot()
        unknown = {node.kind for node in graph.nodes} - implementations.keys()
        if unknown:
            raise GraphError(f'Unregistered node kinds: {sorted(unknown)}')
        inputs = json_data({} if inputs is None else inputs)
        if type(inputs) is not dict:
            raise GraphError('Run inputs must be an object')
        trace = Trace(uuid4().hex, trace_dir)
        self._busy, self._cancel = True, asyncio.Event()
        outputs, final_outputs, visits = {}, {}, {}
        status, message, steps, cleanup_complete = RunStatus.FAILED, '', 0, True
        deadline = time.monotonic() + graph.timeout_sec
        by_id = {node.id: node for node in graph.nodes}
        routes = {(edge.source, edge.on): edge.target for edge in graph.edges}
        current = graph.entry
        try:
            trace.record('run_started', graph=graph.to_dict(), inputs=inputs)
            while True:
                if self._cancel.is_set():
                    status, message = RunStatus.CANCELLED, 'Run cancellation requested'
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    status, message = RunStatus.TIMED_OUT, 'Graph execution deadline exceeded'
                    break
                if steps >= graph.max_steps:
                    message = 'Graph max_steps exceeded'
                    break
                spec = by_id[current]
                steps += 1
                visits[current] = visits.get(current, 0) + 1
                context = NodeContext(trace.run_id, current, steps, visits[current],
                                      self.connector, self._cancel)
                # Never reuse the current node's previous output after a failed visit.
                outputs.pop(current, None)
                trace.record('node_started', node=current, kind=spec.kind, step=steps,
                             visit=context.visit, call_id=context.call_id)
                try:
                    arguments = resolve(spec.inputs, inputs, outputs)
                except GraphError as exc:
                    result = NodeResult('error', message=str(exc))
                else:
                    trace.record('node_inputs', node=current, step=steps, inputs=arguments)
                    result, cleanup_complete = await self._execute_node(
                        implementations[spec.kind], context, arguments,
                        min(spec.timeout_sec, max(0, deadline - time.monotonic())))
                trace.record('node_finished', node=current, step=steps, outcome=result.outcome,
                             outputs=result.outputs, message=result.message,
                             cleanup_complete=cleanup_complete)
                if not cleanup_complete:
                    message = f'Cleanup unconfirmed at {current}: {result.message}'
                    break
                if self._cancel.is_set():
                    status, message = RunStatus.CANCELLED, result.message or 'Run cancellation requested'
                    break
                if result.outcome == 'cancelled':
                    status, message = RunStatus.CANCELLED, result.message
                    break
                if time.monotonic() >= deadline:
                    status, message = RunStatus.TIMED_OUT, 'Graph execution deadline exceeded'
                    break
                # Only successful or custom domain outcomes provide consumable data.
                if result.outcome not in ('error', 'timeout', 'failure'):
                    outputs[current] = json_data(result.outputs)
                if spec.kind == 'end':
                    if result.outcome in ('success', 'failure'):
                        status = RunStatus.SUCCEEDED if result.outcome == 'success' else RunStatus.FAILED
                        final_outputs, message = json_data(result.outputs), result.message
                    else:
                        message = f'End node failed: {result.message}'
                    break
                target = routes.get((current, result.outcome))
                if target is None:
                    status = RunStatus.TIMED_OUT if result.outcome == 'timeout' else RunStatus.FAILED
                    message = f'Unhandled outcome {result.outcome!r} at {current}: {result.message}'
                    break
                trace.record('edge_taken', source=current, target=target, outcome=result.outcome)
                current = target
            trace.record('run_finished', status=status.value, steps=steps, outputs=final_outputs,
                         message=message, cleanup_complete=cleanup_complete)
            return ExecutionResult(trace.run_id, status, steps, final_outputs, json_data(outputs),
                                   message, cleanup_complete, tuple(trace.events),
                                   str(trace.path) if trace.path else None)
        except asyncio.CancelledError:
            trace.record('run_finished', status=RunStatus.CANCELLED.value, steps=steps,
                         message='Caller task cancelled',
                         cleanup_complete=not self._quarantined and not self._pending)
            raise
        finally:
            self._cancel, self._busy = None, False
            trace.close()
