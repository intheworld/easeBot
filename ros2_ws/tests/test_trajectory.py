"""Continuous knots, analytic bounds, no overshoot and shared ROS playback."""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/hei_agent_bridge/src'))
from hei_agent_bridge.trajectory import JointTrajectory
from hei_agent_bridge.grasp.validation import trajectory_samples


def test_continuous_motion_and_faster_than_stop_at_every_point():
    start = np.zeros(6)
    path = [np.full(6, x) for x in np.linspace(.01, .16, 16)]
    t = JointTrajectory(start, path, .1)
    assert np.all(t.velocities[1:-1] > 0)
    assert np.all(t.velocities[[0,-1]] == 0)
    assert np.all(t.accelerations[[0,-1]] == 0)
    assert np.any(abs(t.accelerations[1:-1]) > 0)
    old = sum(max(.4, 1.875*max(abs(b-a))/(.1*.6)) for a,b in zip([start]+path,path))
    assert t.duration < old  # One smooth phase, with a new jerk bound, still beats per-point stops.
    for i in range(len(path)):
        for fraction, knot in [(0.,i),(1.,i+1)]:
            q,v,a = t.evaluate(i,fraction)
            np.testing.assert_allclose(q,t.positions[knot],atol=1e-12)
            np.testing.assert_allclose(v,t.velocities[knot],atol=1e-12)
            np.testing.assert_allclose(a,t.accelerations[knot],atol=1e-12)


def test_random_paths_obey_bounds_and_reversals_stop():
    rng = np.random.default_rng(18)
    for _ in range(20):
        q = rng.uniform(-1,1,(12,6))
        t = JointTrajectory(q[0],q[1:],.1)
        for i in range(11):
            for f in np.linspace(0,1,81):
                p,v,a = t.evaluate(i,f)
                assert np.all(p >= np.minimum(q[i],q[i+1])-1e-12)
                assert np.all(p <= np.maximum(q[i],q[i+1])+1e-12)
                assert max(abs(v)) <= t.speed_limit+1e-12
                assert max(abs(a)) <= t.acceleration_limit+1e-12
        reversal = np.diff(q,axis=0)[:-1]*np.diff(q,axis=0)[1:] <= 0
        assert np.all(t.velocities[1:-1][reversal] == 0)
        samples = list(t.samples())
        assert np.max(np.abs(np.diff([s[2] for s in samples],axis=0))) <= .005+1e-12
        assert samples[-1][0] == pytest.approx(t.duration)


def test_grasp_phases_use_the_same_curve_and_preserve_jaw_stops():
    start = np.zeros(6)
    pre = np.full(6,.1)
    approach = [np.full(6,x) for x in [.11,.12,.13]]
    plan = dict(start_joint_rad=start,start_gripper_rad=0.,pregrasp_joint_rad=pre,
                approach_joint_rad=approach,
                motion_config=dict(joint_speed_rad_s=.1,gripper_open_rad=-4.,gripper_closed_rad=0.))
    samples = list(trajectory_samples(plan))
    assert {phase for _,phase,_,_ in samples}=={'start','open','pregrasp','approach','close'}
    expected = JointTrajectory(pre,approach,.1)
    actual = [(t,q,g) for t,phase,q,g in samples if phase=='approach']
    for (_,_,q), (_,actual_q,grip) in zip(expected.samples(),actual,strict=True):
        np.testing.assert_allclose(q,actual_q)
        assert grip == -4.
    assert actual[-1][0]-actual[0][0] == pytest.approx(expected.duration)
    assert all(np.array_equal(q,approach[-1]) for _,phase,q,_ in samples if phase=='close')


@pytest.mark.parametrize('start,path,speed',[
    ([0]*6,[],.1),([0]*5,[[1]*5],.1),([0]*6,[[float('nan')]*6],.1),
    ([0]*6,[[1]*6],0),([0]*6,[[1]*6],float('nan'))])
def test_invalid_trajectory(start,path,speed):
    with pytest.raises(ValueError):
        JointTrajectory(start,path,speed)


def test_hold_and_duplicate_points():
    t = JointTrajectory([0]*6,[[0]*6,[0]*6,[.1]*6],.1)
    assert np.all(np.diff(t.times)>0)
    assert np.isfinite(t.velocities).all()
    for i in (0,1):
        np.testing.assert_allclose(t.evaluate(i,.5)[0],0.)


def test_matches_quintic_reconstruction_from_ros_endpoint_fields():
    t = JointTrajectory([0.]*6, [[.01]*6,[.025]*6,[.04]*6],.1)
    for i,h in enumerate(t.intervals):
        # Independent polynomial solution: exactly the endpoint data sent to JTC.
        matrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],
                           [1,h,h*h,h**3,h**4,h**5],
                           [0,1,2*h,3*h*h,4*h**3,5*h**4],
                           [0,0,2,6*h,12*h*h,20*h**3]])
        values = np.stack([t.positions[i],t.velocities[i],t.accelerations[i],
                           t.positions[i+1],t.velocities[i+1],t.accelerations[i+1]])
        coefficients = np.linalg.solve(matrix,values)
        for f in np.linspace(0,1,21):
            q = np.array([(f*h)**k for k in range(6)]) @ coefficients
            np.testing.assert_allclose(q,t.evaluate(i,f)[0],atol=1e-12)


def test_dense_collinear_path_has_one_phase_independent_of_ik_density():
    start=np.zeros(6)
    sparse=JointTrajectory(start,[np.full(6,.16)],.6)
    dense=JointTrajectory(start,[np.full(6,x) for x in np.linspace(.001,.16,160)],.6)
    assert dense.smoothing_method=='phase_quintic'
    assert dense.duration==pytest.approx(sparse.duration,rel=.005)  # Conservative hull bound tightens with subdivision.
    assert dense.duration<1.2  # Old 0.1 s per knot floor took >=16 s.
    for i,h in enumerate(dense.intervals):
        jerk=60*np.diff(dense._controls[i],n=3,axis=0)/h**3
        assert np.max(dense._bound(jerk[None]))<=dense.jerk_limit+1e-8


def test_quintic_phase_uses_same_global_time_law_as_zero_command():
    start=np.zeros(6);target=np.full(6,.16)
    t=JointTrajectory(start,[np.full(6,x) for x in [.01,.03,.08,.13,.16]],.6)
    for i in range(len(t.intervals)):
        for f in np.linspace(0,1,25):
            u=(t.times[i]+f*t.intervals[i])/t.duration
            np.testing.assert_allclose(t.evaluate(i,f)[0],target*(10*u**3-15*u**4+6*u**5),atol=1e-11)


def test_reversal_and_hold_paths_have_bounded_jerk():
    for path in ([[.04]*6,[.02]*6,[.08]*6],[[0.]*6,[0.]*6,[.1]*6]):
        t=JointTrajectory([0.]*6,path,.6)
        for c,h in zip(t._controls,t.intervals):
            j=60*np.diff(c,n=3,axis=0)/h**3
            assert np.max(t._bound(j[None]))<=t.jerk_limit+1e-8


def test_recorded_grasp_approach_is_fast_with_bounded_jerk_and_no_overshoot():
    import json
    scene=json.loads((Path(__file__).parent/'fixtures/trajectory_approach_8475.json').read_text())
    t=JointTrajectory(scene['start'],scene['path'],.6)
    assert t.duration<1.8  # Saved old 0.12 rad/s profile required 4.985 seconds.
    assert np.any(abs(t.accelerations[1:-1])>0)
    for i,(c,h) in enumerate(zip(t._controls,t.intervals)):
        j=60*np.diff(c,n=3,axis=0)/h**3
        assert np.max(t._bound(j[None]))<=7.2+1e-8
        for f in np.linspace(0,1,31):
            q,v,a=t.evaluate(i,f)
            assert np.all(q>=np.minimum(t.positions[i],t.positions[i+1])-1e-12)
            assert np.all(q<=np.maximum(t.positions[i],t.positions[i+1])+1e-12)
            assert np.max(abs(v))<=.36+1e-12
            assert np.max(abs(a))<=.72+1e-12


@pytest.mark.parametrize('stage,old_duration,max_duration',[('transfer',33.38345,23.),('retreat',5.94493,4.)])
def test_recorded_complex_transport_avoids_global_timing_slowdown(stage,old_duration,max_duration):
    import json
    data=json.loads((Path(__file__).parent/'fixtures/trajectory_place_4206.json').read_text())[stage]
    t=JointTrajectory(data['start'],data['path'],.6)
    assert t.smoothing_method=='local_monotone_quintic'
    assert t.duration<max_duration<old_duration
    for c,h in zip(t._controls,t.intervals):
        jerk=60*np.diff(c,n=3,axis=0)/h**3
        assert np.max(t._bound(jerk[None]))<=7.2+1e-8
