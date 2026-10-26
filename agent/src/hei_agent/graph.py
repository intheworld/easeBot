"""Data-only policy definition. Loading a graph never imports or executes scripts."""
from copy import deepcopy
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import re
from typing import Any


SCHEMA = 'hei_agent.graph/v1'
_ID = re.compile(r'^[A-Za-z_][A-Za-z0-9_-]*$')


class GraphError(ValueError):
    pass


def json_data(value: Any) -> Any:
    """Copy JSON data and reject NaN, arbitrary objects and non-string keys."""
    def check(item):
        if item is None or type(item) in (str, bool, int):
            return
        if type(item) is float and math.isfinite(item):
            return
        if type(item) is list:
            for child in item:
                check(child)
            return
        if type(item) is dict and all(type(key) is str for key in item):
            for child in item.values():
                check(child)
            return
        raise GraphError('Expected finite JSON data')
    check(value)
    return deepcopy(value)


def positive_seconds(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise GraphError(f'{name} must be finite and positive')


def references(value):
    if isinstance(value, dict):
        if '$ref' in value:
            if set(value) != {'$ref'} or not isinstance(value['$ref'], str):
                raise GraphError('A reference must be exactly {"$ref": "path"}')
            yield value['$ref']
        else:
            for child in value.values():
                yield from references(child)
    elif isinstance(value, list):
        for child in value:
            yield from references(child)


@dataclass(frozen=True)
class Node:
    """An invocation of a registered implementation, not a ROS node."""
    id: str
    kind: str
    inputs: dict = field(default_factory=dict)
    timeout_sec: float = 30.0


@dataclass(frozen=True)
class Edge:
    source: str
    target: str
    on: str = 'success'


@dataclass(frozen=True)
class Graph:
    name: str
    entry: str
    nodes: tuple[Node, ...]
    edges: tuple[Edge, ...]
    max_steps: int = 100
    timeout_sec: float = 300.0
    catalog_revision: str | None = None

    def validate(self):
        if not isinstance(self.name, str) or not self.name.strip():
            raise GraphError('Graph name must be nonempty')
        positive_seconds(self.timeout_sec, 'graph timeout_sec')
        if self.catalog_revision is not None and (not isinstance(self.catalog_revision, str) or
                not re.fullmatch(r'[0-9a-f]{64}', self.catalog_revision)):
            raise GraphError('catalog_revision must be a SHA256 hex digest')
        if type(self.max_steps) is not int or self.max_steps <= 0:
            raise GraphError('max_steps must be a positive integer')
        by_id = {}
        for node in self.nodes:
            if not isinstance(node.id, str) or not _ID.fullmatch(node.id):
                raise GraphError(f'Invalid node id: {node.id!r}')
            if node.id in by_id:
                raise GraphError(f'Duplicate node: {node.id}')
            if not isinstance(node.kind, str) or not node.kind.strip():
                raise GraphError(f'Invalid node kind: {node.id}')
            if type(node.inputs) is not dict:
                raise GraphError(f'Node inputs must be an object: {node.id}')
            json_data(node.inputs)
            positive_seconds(node.timeout_sec, f'{node.id} timeout_sec')
            if node.kind == 'end':
                if node.inputs.get('status') not in ('succeeded', 'failed'):
                    raise GraphError(f'End node {node.id} needs a literal succeeded/failed status')
            by_id[node.id] = node
        if not isinstance(self.entry, str) or self.entry not in by_id:
            raise GraphError('Entry must name an existing node')
        routes, forward, reverse = set(), {key: set() for key in by_id}, {key: set() for key in by_id}
        for edge in self.edges:
            if not isinstance(edge.source, str) or not isinstance(edge.target, str):
                raise GraphError('Edge endpoints must be node ids')
            if edge.source not in by_id or edge.target not in by_id:
                raise GraphError(f'Unknown edge endpoint: {edge}')
            if by_id[edge.source].kind == 'end':
                raise GraphError('End nodes cannot have outgoing edges')
            if not isinstance(edge.on, str) or not edge.on or (edge.source, edge.on) in routes:
                raise GraphError(f'Empty or ambiguous outcome route: {edge.source}')
            routes.add((edge.source, edge.on))
            forward[edge.source].add(edge.target)
            reverse[edge.target].add(edge.source)
        for node in self.nodes:
            if node.kind != 'end' and not forward[node.id]:
                raise GraphError(f'Nonterminal node has no outgoing edge: {node.id}')
            for ref in references(node.inputs):
                parts = ref.split('.')
                if parts[0] == 'inputs' and all(parts[1:]):
                    continue
                if (len(parts) >= 2 and parts[0] == 'nodes' and
                        parts[1] in by_id and all(parts[1:])):
                    continue
                raise GraphError(f'Invalid data reference: {ref}')

        def walk(seeds, adjacency):
            seen, todo = set(), list(seeds)
            while todo:
                item = todo.pop()
                if item not in seen:
                    seen.add(item)
                    todo.extend(adjacency[item])
            return seen

        if walk([self.entry], forward) != set(by_id):
            raise GraphError('Graph contains unreachable nodes')
        ends = [node.id for node in self.nodes if node.kind == 'end']
        if walk(ends, reverse) != set(by_id):
            raise GraphError('Every node must have a possible path to an end node')
        return self

    def to_dict(self):
        return json_data({
            'schema': SCHEMA, 'name': self.name, 'entry': self.entry,
            'max_steps': self.max_steps, 'timeout_sec': self.timeout_sec,
            'nodes': [{'id': n.id, 'kind': n.kind, 'inputs': n.inputs,
                       'timeout_sec': n.timeout_sec} for n in self.nodes],
            'edges': [{'source': e.source, 'target': e.target, 'on': e.on} for e in self.edges],
            **({'catalog_revision': self.catalog_revision} if self.catalog_revision is not None else {}),
        })

    @classmethod
    def from_dict(cls, data):
        data = json_data(data)
        if type(data) is not dict or data.get('schema') != SCHEMA:
            raise GraphError(f'Expected schema {SCHEMA}')
        try:
            unknown = set(data) - {'schema', 'name', 'entry', 'nodes', 'edges', 'max_steps', 'timeout_sec', 'catalog_revision'}
            if unknown:
                raise GraphError(f'Unknown graph fields: {sorted(unknown)}')
            if type(data['nodes']) is not list or type(data['edges']) is not list:
                raise GraphError('nodes and edges must be arrays')
            graph = cls(
                name=data['name'], entry=data['entry'],
                nodes=tuple(Node(**node) for node in data['nodes']),
                edges=tuple(Edge(**edge) for edge in data['edges']),
                max_steps=data.get('max_steps', 100), timeout_sec=data.get('timeout_sec', 300.0),
                catalog_revision=data.get('catalog_revision'),
            )
            return graph.validate()
        except (TypeError, KeyError) as exc:
            raise GraphError(f'Malformed graph: {exc}') from exc

    @classmethod
    def load(cls, path: str | Path):
        return cls.loads(Path(path).read_text())

    @classmethod
    def loads(cls, text: str):
        """Parse strict graph JSON, including duplicate-key detection."""
        def unique_keys(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise GraphError(f'Duplicate JSON key: {key}')
                result[key] = value
            return result
        return cls.from_dict(json.loads(text, object_pairs_hook=unique_keys))


def resolve(value, inputs, outputs):
    """Resolve dictionary paths; a skipped/unexecuted node is an error, not None."""
    if isinstance(value, dict):
        if '$ref' in value:
            ref = value['$ref']
            current = {'inputs': inputs, 'nodes': outputs}
            for part in ref.split('.'):
                if not isinstance(current, dict) or part not in current:
                    raise GraphError(f'Unavailable data reference: {ref}')
                current = current[part]
            return json_data(current)
        return {key: resolve(child, inputs, outputs) for key, child in value.items()}
    if isinstance(value, list):
        return [resolve(child, inputs, outputs) for child in value]
    return value
