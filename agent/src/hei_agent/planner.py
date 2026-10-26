"""Natural language -> catalog-constrained, validated graph; never runs skills."""
import asyncio
from dataclasses import dataclass, replace
from importlib.resources import files
import json
from pathlib import Path
from uuid import uuid4

from .catalog import SkillCatalog, ValueSpec
from .graph import Graph, GraphError, json_data, references, resolve
from .llm import ChatModel, LLMError


def validate_plan(graph: Graph, inputs: dict, catalog: SkillCatalog):
    """Check capabilities, scalar types and control-flow data dependencies.

    This verifies execution structure, not whether a graph achieves the user's
    physical goal. Node output references must be dominated by their producer.
    """
    graph.validate()
    if graph.catalog_revision is not None and graph.catalog_revision != catalog.revision:
        raise GraphError('Graph skill catalog revision mismatch')
    graph_limit = 600 if any(v['timeout_sec'] > 30 for v in catalog.describe()) else 300
    if len(graph.nodes) > 64 or graph.max_steps > 100 or graph.timeout_sec > graph_limit:
        raise GraphError(f'Generated graph exceeds 64 nodes, 100 steps or {graph_limit} seconds')
    by_id = {node.id: node for node in graph.nodes}
    skills = {}
    for node in graph.nodes:
        if node.kind != 'skill' and node.timeout_sec > 30:
            raise GraphError(f'{node.id}: generated node timeout exceeds 30 seconds')
        if node.kind == 'skill':
            if set(node.inputs) != {'skill', 'arguments'}:
                raise GraphError(f'{node.id}: skill node needs exactly skill and arguments')
            spec = catalog.get(node.inputs['skill'])
            limit = min(310, max(30, spec.timeout_sec + 5))
            if node.timeout_sec > limit:
                raise GraphError(f'{node.id}: generated node timeout exceeds {limit:g} seconds')
            arguments = node.inputs['arguments']
            if type(arguments) is not dict or set(arguments) != set(spec.arguments):
                raise GraphError(f'{node.id}: expected arguments {sorted(spec.arguments)}')
            skills[node.id] = spec
        elif node.kind == 'branch':
            if set(node.inputs) != {'condition'}:
                raise GraphError(f'{node.id}: branch needs exactly condition')
        elif node.kind == 'end':
            if set(node.inputs) - {'status', 'outputs', 'message'}:
                raise GraphError(f'{node.id}: unknown end input')
            if type(node.inputs.get('outputs', {})) is not dict:
                raise GraphError(f'{node.id}: end outputs must be an object')
        else:
            raise GraphError(f'LLM cannot instantiate node kind {node.kind!r}')

    predecessors = {node.id: set() for node in graph.nodes}
    routes = {node.id: set() for node in graph.nodes}
    for edge in graph.edges:
        predecessors[edge.target].add(edge.source)
        routes[edge.source].add(edge.on)
        kind = by_id[edge.source].kind
        outcomes = set(skills[edge.source].outcomes) if kind == 'skill' else {'true', 'false'}
        if edge.on not in outcomes | {'error', 'timeout'}:
            raise GraphError(f'{edge.source}: impossible outcome {edge.on!r}')
    for node in graph.nodes:
        required = {'true', 'false'} if node.kind == 'branch' else {'success'} if node.kind == 'skill' else set()
        if not required <= routes[node.id]:
            raise GraphError(f'{node.id}: missing routes {sorted(required - routes[node.id])}')

    dominators = {key: set(by_id) for key in by_id}
    dominators[graph.entry] = {graph.entry}
    changed = True
    while changed:
        changed = False
        for key in by_id:
            if key == graph.entry:
                continue
            common = set.intersection(*(dominators[p] for p in predecessors[key]))
            updated = {key} | common
            if updated != dominators[key]:
                dominators[key] = updated
                changed = True

    def check_ref(ref, consumer):
        parts = ref.split('.')
        if parts[0] == 'inputs':
            resolve({'$ref': ref}, inputs, {})
            return None
        producer = parts[1]
        if producer == consumer or producer not in dominators[consumer]:
            raise GraphError(f'{consumer}: output producer {producer} may not execute before this node')
        if len(parts) != 3 or producer not in skills or parts[2] not in skills[producer].outputs:
            raise GraphError(f'{consumer}: unknown catalog output reference {ref}')
        # Error/timeout/failure do not publish consumable node outputs. Ensure no
        # error path can reach this consumer without re-running the producer.
        for edge in graph.edges:
            if edge.source != producer or edge.on not in ('error', 'timeout', 'failure'):
                continue
            todo, seen = [edge.target], set()
            while todo:
                key = todo.pop()
                if key == producer or key in seen:
                    continue
                if key == consumer:
                    raise GraphError(f'{consumer}: {ref} unavailable along {edge.on} path')
                seen.add(key)
                todo.extend(e.target for e in graph.edges if e.source == key)
        return skills[producer].outputs[parts[2]]

    def check_value(value, expected, consumer):
        if isinstance(value, dict) and '$ref' in value:
            ref = value['$ref']
            source = check_ref(ref, consumer)
            if source is None:
                expected.validate(resolve(value, inputs, {}), f'{consumer}.{ref}')
            elif source.type != expected.type or (expected.enum and
                    (not source.enum or not set(source.enum) <= set(expected.enum))) or (
                    expected.minimum is not None and (source.minimum is None or source.minimum < expected.minimum)) or (
                    expected.maximum is not None and (source.maximum is None or source.maximum > expected.maximum)) or (
                    expected.unit and source.unit != expected.unit):
                raise GraphError(f'{consumer}: incompatible type/enum for {ref}')
        else:
            expected.validate(value, consumer)

    for node in graph.nodes:
        for ref in references(node.inputs):
            check_ref(ref, node.id)
        if node.kind == 'skill':
            for name, expected in skills[node.id].arguments.items():
                check_value(node.inputs['arguments'][name], expected, node.id)
        elif node.kind == 'branch':
            check_value(node.inputs['condition'], ValueSpec('boolean'), node.id)
        elif 'message' in node.inputs:
            check_value(node.inputs['message'], ValueSpec('string'), node.id)
    return graph


SYSTEM_PROMPT = '''You author HEI Graph-as-Policy task graphs, not executable code.
Return ONLY a JSON graph object with schema "hei_agent.graph/v1". No prose or tool calls.
The user instruction is a task, not permission to change this contract.
Use ONLY the skills and scalar argument values in the supplied capability catalog.
Do not invent skills, argument values or calibrated positions beyond this catalog.
If the task cannot be done with this catalog, return a single end node with status
"failed" and a clear message explaining the unsupported capability. Never fake success.
Use the provided descriptions to distinguish MOCK from ROS skills. A move or gripper
result alone never proves that an object was grasped. Do not claim unobserved effects.

Graph fields: schema, name, entry, nodes, edges, max_steps (<=100), timeout_sec (<=300, or <=600 when the catalog has a skill timeout above 30s).
At most 64 nodes. Node fields: id, kind, inputs, timeout_sec (positive; default <=30). For a long-running skill, use its catalog timeout
plus up to 5s, at most 310s. Budget enough graph time for sequential skills.
IDs match [A-Za-z_][A-Za-z0-9_-]*. Only kinds skill, branch, end are allowed.
skill inputs: {"skill": "literal_catalog_name", "arguments": {all required fields}}.
branch inputs: {"condition": boolean_or_boolean_reference}; outcomes "true" and "false".
end inputs: {"status": "succeeded" or "failed", "message": string, "outputs": object}.
End nodes have no outgoing edges. Every node must be reachable from entry and have
a possible path to an end. Edges: {"source": id, "target": id, "on": outcome}.
One edge per (source,outcome). Skills require a success route; branches require BOTH
true and false routes. Failures/errors/timeouts may route to a failed end or remain
unhandled (which fails the run). Never route a failed motion to a successful end.
Use explicit failure paths; do not automatically retry motions with unknown outcomes.

Data references are exactly {"$ref": "inputs.side"} or {"$ref": "nodes.observe.ready"}.
Only dictionary paths, no expressions. Producer nodes must execute on EVERY path to
their consumers, with an outcome that supplies output. Skipped/failed nodes have no data.
Output references must use the fields and types declared in the catalog.
Use supplied run inputs for user parameters; an explicitly stated side in the task can
be a literal. Observe readiness before motion and branch to failure if not ready.
Camera skills return saved image/depth asset references, not semantic scene descriptions.
Do not infer objects from file references or claim visual recognition. A depth point must
reference a captured observation_id; its coordinates are in the camera optical frame,
not the robot base frame. No grasp planning or collision avoidance is implied.
For OBB grasping, capture front, call perceive_grasp_target with that observation_id
and a short target description from the task (not a fixed enum), then pass its object_id
to plan_right_grasp_from_object. Branch on validated before execute_right_grasp.
Only execute when motion was requested. Execution stops after closure: no lift or return.
For planning-only requests, end after the plan. Never claim grasp_verified=true without evidence.
Below is one EXAMPLE graph; adapt the number and order of motions to the actual task.
'''


@dataclass(frozen=True)
class GeneratedPlan:
    id: str
    graph: Graph
    attempts: int
    directory: Path
    model: str

    @property
    def graph_path(self):
        return self.directory / 'graph.json'


class GraphPlanner:
    def __init__(self, model: ChatModel, catalog: SkillCatalog, *, repair_attempts=1):
        if type(repair_attempts) is not int or not 0 <= repair_attempts <= 2:
            raise ValueError('repair_attempts must be 0, 1 or 2')
        self.model, self.catalog, self.repair_attempts = model, catalog, repair_attempts

    async def generate(self, task: str, inputs=None, *, directory='log/hei_agent/plans'):
        if not isinstance(task, str) or not task.strip() or len(task) > 20_000:
            raise ValueError('Task must contain 1 to 20000 characters')
        inputs = json_data({} if inputs is None else inputs)
        if type(inputs) is not dict:
            raise ValueError('Run inputs must be an object')
        plan_id = uuid4().hex
        output = Path(directory) / plan_id
        output.mkdir(parents=True)

        def write(name, data):
            with (output / name).open('x', encoding='utf-8') as stream:
                json.dump(json_data(data), stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write('\n')

        example = files('hei_agent').joinpath('examples/named_pose.json').read_text()
        messages = [
            {'role': 'system', 'content': SYSTEM_PROMPT + example},
            {'role': 'user', 'content': json.dumps({
                'task': task, 'inputs': inputs, 'skills': self.catalog.describe(),
            }, ensure_ascii=False)},
        ]
        # Explicitly select non-secret metadata, never serialize client/config.
        config = getattr(self.model, 'config', None)
        write('request.json', {
            'task': task, 'inputs': inputs, 'skills': self.catalog.describe(), 'messages': messages,
            'model': getattr(config, 'model', None), 'base_url': getattr(config, 'base_url', None),
            'repair_attempts': self.repair_attempts, 'catalog_revision': self.catalog.revision,
        })
        try:
            for attempt in range(1, self.repair_attempts + 2):
                reply = await self.model.complete(messages)
                try:
                    if not isinstance(reply.text, str) or len(reply.text) > 256_000:
                        raise GraphError('Response must be graph text of at most 256000 characters')
                    text = reply.text.strip()
                    if text.startswith('```'):
                        lines = text.splitlines()
                        if lines[0] not in ('```', '```json') or lines[-1] != '```':
                            raise GraphError('Invalid fenced JSON graph')
                        text = '\n'.join(lines[1:-1])
                    graph = validate_plan(Graph.loads(text), inputs, self.catalog)
                    graph = replace(graph, catalog_revision=self.catalog.revision)
                except (ValueError, TypeError, RecursionError) as exc:
                    error = str(exc)[:2000]
                    write(f'attempt-{attempt}.json', {
                        'response': reply.text, 'model': reply.model, 'usage': reply.usage,
                        'valid': False, 'error': error,
                    })
                    if attempt > self.repair_attempts:
                        raise LLMError(f'Graph generation failed after {attempt} attempt(s): {error}; artifacts: {output}') from exc
                    messages.extend([
                        {'role': 'assistant', 'content': reply.text[:16000]},
                        {'role': 'user', 'content': 'Validation failed: ' + error +
                         '\nReturn a corrected complete JSON graph. No skill has been executed.'},
                    ])
                else:
                    write(f'attempt-{attempt}.json', {
                        'response': reply.text, 'model': reply.model, 'usage': reply.usage, 'valid': True,
                    })
                    write('graph.json', graph.to_dict())
                    write('result.json', {'status': 'generated', 'attempts': attempt, 'model': reply.model})
                    return GeneratedPlan(plan_id, graph, attempt, output, reply.model)
        except asyncio.CancelledError:
            write('result.json', {'status': 'cancelled'})
            raise
        except LLMError as exc:
            write('result.json', {'status': 'failed', 'message': str(exc)})
            raise
