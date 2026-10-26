"""Small typed capability catalog shared by planning and the mock connector.

Only flat, required scalar fields are supported in v1. This is deliberately not a
partial implementation of arbitrary JSON Schema validation.
"""
from dataclasses import dataclass, field
import hashlib
import json

from .graph import GraphError, json_data, positive_seconds


@dataclass(frozen=True)
class ValueSpec:
    type: str
    enum: tuple = ()
    minimum: float | None = None
    maximum: float | None = None
    unit: str = ''

    def __post_init__(self):
        if self.type not in ('string', 'boolean', 'number'):
            raise ValueError(f'Unsupported catalog value type: {self.type}')
        if not isinstance(self.unit, str):
            raise GraphError('Catalog units must be strings')
        for value in self.enum:
            self.validate(value, 'enum')
        for limit in (self.minimum, self.maximum):
            if limit is not None:
                if self.type != 'number' or type(limit) not in (int, float):
                    raise ValueError('Numeric limits require number fields')
                json_data(limit)
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError('minimum exceeds maximum')

    def validate(self, value, name):
        valid_type = {
            'string': type(value) is str,
            'boolean': type(value) is bool,
            'number': type(value) in (int, float),
        }[self.type]
        json_data(value)
        if not valid_type or (self.enum and value not in self.enum):
            raise GraphError(f'{name}: expected {self.type}' + (f' in {self.enum}' if self.enum else ''))
        if self.type == 'number' and ((self.minimum is not None and value < self.minimum) or
                                      (self.maximum is not None and value > self.maximum)):
            raise GraphError(f'{name}: outside declared numeric range')

    def to_dict(self):
        return {'type': self.type, **({'enum': list(self.enum)} if self.enum else {}),
                **({'minimum': self.minimum} if self.minimum is not None else {}),
                **({'maximum': self.maximum} if self.maximum is not None else {}),
                **({'unit': self.unit} if self.unit else {})}


@dataclass(frozen=True)
class SkillSpec:
    name: str
    description: str
    arguments: dict[str, ValueSpec] = field(default_factory=dict)
    outputs: dict[str, ValueSpec] = field(default_factory=dict)
    outcomes: tuple[str, ...] = ('success',)
    version: str = '1'
    motion: bool = False
    timeout_sec: float = 10.0

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name or not isinstance(self.description, str):
            raise GraphError('Skill name and description must be strings')
        if not isinstance(self.version, str) or not self.version or type(self.motion) is not bool:
            raise GraphError('Invalid skill version/motion declaration')
        positive_seconds(self.timeout_sec, 'skill timeout_sec')
        if set(self.arguments) & {'revision', 'invocation_id'}:
            raise GraphError('Skill arguments conflict with MHS invocation identity fields')
        if not self.outcomes or 'success' not in self.outcomes or any(
                not isinstance(value, str) or value in ('error', 'timeout', 'cancelled') for value in self.outcomes):
            raise GraphError('Invalid skill outcome declaration')

    def validate_arguments(self, arguments):
        if type(arguments) is not dict or set(arguments) != set(self.arguments):
            raise GraphError(f'{self.name} requires exactly these arguments: {sorted(self.arguments)}')
        for name, spec in self.arguments.items():
            spec.validate(arguments[name], f'{self.name}.{name}')

    def validate_outputs(self, outputs):
        if type(outputs) is not dict or set(outputs) != set(self.outputs):
            raise GraphError(f'{self.name}: output fields do not match catalog')
        for key, spec in self.outputs.items():
            spec.validate(outputs[key], f'{self.name}.outputs.{key}')

    def to_dict(self):
        return {
            'name': self.name, 'description': self.description,
            'arguments': {key: value.to_dict() for key, value in self.arguments.items()},
            'outputs': {key: value.to_dict() for key, value in self.outputs.items()},
            'outcomes': list(self.outcomes),
            'version': self.version, 'motion': self.motion, 'timeout_sec': self.timeout_sec,
        }


class SkillCatalog:
    def __init__(self, skills, *, revision=None, backend=None):
        self.revision = revision
        self.backend = backend
        self._skills = {}
        for skill in skills:
            if skill.name in self._skills:
                raise ValueError(f'Duplicate skill: {skill.name}')
            self._skills[skill.name] = skill

    def get(self, name):
        if not isinstance(name, str) or name not in self._skills:
            raise GraphError(f'Unknown skill: {name!r}')
        return self._skills[name]

    def describe(self):
        return [skill.to_dict() for skill in self._skills.values()]

    @classmethod
    def from_manifest(cls, manifest):
        manifest = json_data(manifest)
        if (type(manifest) is not dict or set(manifest) != {'schema', 'backend', 'configuration_digest', 'skills', 'revision'}
                or manifest['schema'] != 'hei.skills/v1' or type(manifest['skills']) is not list
                or manifest['backend'] not in ('mock', 'ros')):
            raise GraphError('Invalid remote skill manifest')
        payload = {key: value for key, value in manifest.items() if key != 'revision'}
        digest = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':'),
                                          ensure_ascii=False, allow_nan=False).encode()).hexdigest()
        if manifest['revision'] != digest:
            raise GraphError('Skill manifest digest mismatch')
        skills = []
        try:
            for entry in manifest['skills']:
                fields = {}
                for group in ('arguments', 'outputs'):
                    fields[group] = {name: ValueSpec(**{**spec, 'enum': tuple(spec.get('enum', ()))})
                                     for name, spec in entry[group].items()}
                skills.append(SkillSpec(**{**entry, **fields, 'outcomes': tuple(entry['outcomes'])}))
        except (TypeError, KeyError, AttributeError) as exc:
            raise GraphError('Malformed remote skill definition') from exc
        return cls(skills, revision=digest, backend=manifest['backend'])


MOCK_CATALOG = SkillCatalog((
    SkillSpec('get_robot_state', 'Read mock readiness; does not move anything.',
              outputs={'ready': ValueSpec('boolean')}),
    SkillSpec('move_to_named_pose',
              'Simulate reaching a named pose. These names have no real robot calibration.',
              arguments={'side': ValueSpec('string', ('left', 'right')),
                         'pose': ValueSpec('string', ('pre_grasp', 'transport'))},
              outputs={'side': ValueSpec('string', ('left', 'right')),
                       'pose': ValueSpec('string', ('pre_grasp', 'transport'))},
              outcomes=('success', 'failure')),
), backend='mock')
