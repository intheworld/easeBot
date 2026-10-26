"""In-memory skills for exercising policy execution; never imports ROS or MHS."""
import asyncio
import math

from .graph import json_data
from .catalog import MOCK_CATALOG
from .node import NodeResult


class MockConnector:
    catalog = MOCK_CATALOG

    def __init__(self, *, ready=True, delay_sec=0.01, fail_at=None):
        if type(delay_sec) not in (int, float) or not math.isfinite(delay_sec) or delay_sec < 0:
            raise ValueError('delay_sec must be finite and nonnegative')
        self.ready = ready
        self.delay_sec = delay_sec
        self.fail_at = fail_at
        self.calls = []
        self.cancelled_calls = set()
        self.poses = {}

    async def call(self, skill, arguments, *, call_id):
        if call_id in self.cancelled_calls:
            raise RuntimeError('Invocation was cancelled before dispatch')
        self.catalog.get(skill).validate_arguments(arguments)
        self.calls.append({'call_id': call_id, 'skill': skill, 'arguments': json_data(arguments)})
        await asyncio.sleep(self.delay_sec)
        if skill == 'get_robot_state':
            return NodeResult(outputs={'ready': self.ready})
        if skill == 'move_to_named_pose':
            pose = arguments['pose']
            if not self.ready or pose == self.fail_at:
                return NodeResult('failure', message='Simulated motion failure')
            self.poses[arguments['side']] = pose
            return NodeResult(outputs={'side': arguments['side'], 'pose': pose})
        raise ValueError(f'Unknown mock skill: {skill}')

    async def cancel(self, call_id):
        # There is no external actuator. The executor also waits for the call
        # coroutine to finish before it permits another step.
        self.cancelled_calls.add(call_id)
