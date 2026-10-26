"""Encoder evidence distinguishes stationary DM velocity spikes from base motion."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.ros_skills import RosSkillBackend
import hei_agent_bridge.ros_skills as module

WHEELS = ['front_right_wheel_joint','rear_right_wheel_joint','rear_left_wheel_joint','front_left_wheel_joint']


def backend(monkeypatch):
    client = SimpleNamespace(server_is_ready=lambda:True)
    b = RosSkillBackend.__new__(RosSkillBackend)
    b.arm, b.grippers = {'right':client}, {'right':client}
    b.samples, b.stamps, b.wheel_history, b.lock = {}, {}, {}, threading.Lock()
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr(module.time, 'monotonic', lambda:clock.now)
    for name in b.joints('right')+b.joints('left')+['lift_joint','right_gripper_motor','left_gripper_motor']:
        b.samples[name] = (0.,0.,clock.now)
    return b, clock


def feed(b, clock, mode):
    for i in range(11):
        clock.now = 100.+i*.05
        p, v = 0., -.0073261261
        if mode == 'spike' and i == 10: p,v = -.0003814697,-.0805854797
        if mode == 'position_motion': p = i*.002
        if mode == 'velocity_motion': v = .1
        b._update('base', (1,i), [(name,p,v) for name in WHEELS])


def test_stationary_quantized_velocity_spike_is_accepted(monkeypatch):
    b, clock = backend(monkeypatch)
    feed(b,clock,'spike')
    assert abs(b.samples[WHEELS[0]][1]) > .05
    state = b.grasp_state()
    assert len(state['wheels']) == 4


@pytest.mark.parametrize('mode', ['position_motion','velocity_motion'])
def test_real_motion_is_rejected_even_if_another_signal_is_static(monkeypatch, mode):
    b, clock = backend(monkeypatch)
    feed(b,clock,mode)
    with pytest.raises(RuntimeError, match='Grasp scene is moving: wheels; front_right_wheel_joint'):
        b.grasp_state(at_rest=False)


@pytest.mark.parametrize('mode', ['startup','stale','duplicate_stamp'])
def test_stationarity_requires_fresh_distinct_feedback(monkeypatch, mode):
    b, clock = backend(monkeypatch)
    if mode == 'startup':
        b._update('base',(1,0),[(n,0.,0.) for n in WHEELS])
    elif mode == 'stale':
        feed(b,clock,'steady');clock.now += .3
    else:
        for i in range(11):
            clock.now=100.+i*.05
            b._update('base',(1,0),[(n,0.,0.) for n in WHEELS])
    with pytest.raises(RuntimeError, match='stationary wheel history unavailable'):
        b.grasp_state()


def test_left_arm_motion_is_not_hidden_by_wheel_filter(monkeypatch):
    b, clock = backend(monkeypatch)
    feed(b,clock,'spike')
    b.samples['b_left_joint1']=(0.,.1,clock.now)
    with pytest.raises(RuntimeError, match='Grasp scene is moving: left'):
        b.grasp_state(at_rest=False)
