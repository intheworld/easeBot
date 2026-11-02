"""Placement can approximate interior samples without relaxing endpoint checks."""
import numpy as np
import pytest
from hei_agent_bridge.grasp.planner import RightArmPlanner


class Planner(RightArmPlanner):
    def __init__(self, fail_endpoint=False, jump=False):
        self.calls = []
        self.fail_endpoint = fail_endpoint
        self.jump = jump
        self.fk = lambda q: np.eye(4)

    def solve(self, target, seed, *, require_pose_accuracy=True):
        self.calls.append(require_pose_accuracy)
        endpoint = target[0, 3] == .015
        if require_pose_accuracy and (not endpoint or self.fail_endpoint):
            raise ValueError('Right-arm IK failed')
        return np.asarray(seed) + (.3 if self.jump else .01)


def goal():
    t = np.eye(4)
    t[0, 3] = .015
    return t


def test_placement_interior_approximate_endpoint_strict():
    p = Planner()
    assert len(p.cartesian_path(np.zeros(6), goal(), strict_intermediate=False)) == 3
    assert p.calls == [False, False, True]


def test_other_paths_keep_strict_interior():
    with pytest.raises(ValueError, match='IK failed'):
        Planner().cartesian_path(np.zeros(6), goal())


def test_placement_endpoint_failure_is_preserved():
    with pytest.raises(ValueError, match='IK failed'):
        Planner(fail_endpoint=True).cartesian_path(np.zeros(6), goal(), strict_intermediate=False)


def test_placement_branch_jump_is_preserved():
    with pytest.raises(ValueError, match='branch jump'):
        Planner(jump=True).cartesian_path(np.zeros(6), goal(), strict_intermediate=False)


def test_generated_transit_endpoint_is_approximate():
    p = Planner(fail_endpoint=True)
    assert len(p.cartesian_path(np.zeros(6), goal(), strict_intermediate=False,
                                strict_endpoint=False)) == 3
    assert p.calls == [False, False, False]


def test_path_cache_separates_endpoint_accuracy():
    from hei_agent_bridge.grasp.planner_backend import cached_cartesian_path
    p = Planner(fail_endpoint=True)
    cached_cartesian_path(p, np.zeros(6), goal(), strict_intermediate=False, strict_endpoint=False)
    with pytest.raises(ValueError, match='IK failed'):
        cached_cartesian_path(p, np.zeros(6), goal(), strict_intermediate=False)


def test_placement_checks_only_last_waypoint_accuracy(tmp_path, monkeypatch):
    from test_arm_motion import make_arm_service, URDF
    from hei_agent_bridge.arm.geometry import pose_fields
    from hei_agent_bridge.arm.planner import make_plan
    from hei_agent_bridge.grasp import planner_backend
    service, _, backend = make_arm_service(tmp_path)
    start = service.fk(backend.state['right'])
    goals = []
    for i in range(3):
        pose = start.copy(); pose[0, 3] += .001*(i+1)
        goals.append(pose_fields(pose, 'a_right_base_link'))
    calls = []
    def path(planner, seed, goal, **flags):
        calls.append(flags)
        return [np.asarray(seed).copy()]
    monkeypatch.setattr(planner_backend, 'cached_cartesian_path', path)
    request = dict(urdf=str(URDF),state=backend.state,profile=service.profile,
                   arguments=dict(side='right',frame_id='a_right_base_link',path_mode='linear',
                                  _placement_soft_intermediate=True,waypoints=goals))
    make_plan(request)
    assert calls == [dict(strict_intermediate=False,strict_endpoint=False),
                     dict(strict_intermediate=False,strict_endpoint=False),
                     dict(strict_intermediate=False,strict_endpoint=True)]
