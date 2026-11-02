"""Settling/reacquisition and timestamp ownership; no ROS or physical devices."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/hei_agent_bridge/src'))

import pytest

from hei_agent_bridge.joint_history import JointHistory
from test_wrist_refinement import enable_wrist,make_obb_plan
from test_grasp_skills import make_service,invoke


def test_joint_interpolation_requires_two_close_fresh_samples_and_rejects_drift(monkeypatch):
    monkeypatch.setattr('hei_agent_bridge.joint_history.time.monotonic',lambda:10.)
    h=JointHistory();h.append(100.,[0.]*6,9.98);h.append(100.02,[.002]*6,10.)
    result=h.at(100.01,max_skew_sec=.03,max_span_rad=.003)
    assert result['right']==pytest.approx([.001]*6)
    assert result['method']=='ros_stamp_interpolation'
    assert h.at(100.03,max_skew_sec=.03,max_span_rad=.003) is None  # No extrapolation.
    assert h.at(100.01,max_skew_sec=.005,max_span_rad=.003) is None
    assert h.at(100.01,max_skew_sec=.03,max_span_rad=.001) is None
    monkeypatch.setattr('hei_agent_bridge.joint_history.time.monotonic',lambda:12.)
    assert h.at(100.01,max_skew_sec=.03,max_span_rad=.003) is None


def test_history_is_bounded_and_clock_reset_does_not_mix_epochs(monkeypatch):
    monkeypatch.setattr('hei_agent_bridge.joint_history.time.monotonic',lambda:10.)
    h=JointHistory()
    for i in range(2000):h.append(100+i*.001,[0.]*6,8+i*.001)
    assert len(h.samples)==1024
    h.append(1.,[1.]*6,10.)
    assert len(h.samples)==1 and h.samples[0][0]==1.


def test_stability_uses_all_feedback_in_window_including_between_poll_jitter(monkeypatch):
    monkeypatch.setattr('hei_agent_bridge.joint_history.time.monotonic',lambda:10.)
    h=JointHistory()
    for i in range(51):h.append(100+i*.01,[.01 if i==35 else 0.]*6,9.5+i*.01)
    value=h.window(.4)
    assert value['sample_count']>=40 and value['joint_span_rad']==[.01]*6
    monkeypatch.setattr('hei_agent_bridge.joint_history.time.monotonic',lambda:10.2)
    assert h.window(.4) is None


@pytest.mark.parametrize('when',['capture','compute'])
def test_small_convergence_motion_is_reacquired_not_executed_from_stale_pose(tmp_path,when):
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        config=service.profile['wrist_refinement'];config['capture_retries']=2
        capture=service.cameras.capture;captures=[]
        async def moving_capture(args,invocation):
            result=await capture(args,invocation);captures.append(result)
            if when=='capture' and len(captures)==2:backend.state['right'][2]+=.004
            return result
        service.cameras.capture=moving_capture
        original=service._worker;requests=[]
        async def worker(directory):
            request=json.loads((directory/'request.json').read_text())
            if request['operation']!='refine_wrist':return await original(directory)
            requests.append(request)
            if when=='compute' and len(requests)==1:backend.state['right'][2]+=.004
            plan=deepcopy(request['reference_plan'])
            return dict(plan=plan,reference_plan=plan,validation={'status':'passed'},converged=True)
        service._worker=worker;service.binding=service._binding()
        planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':planned['outputs']['plan_id']},'recover')
        assert result['status']=='succeeded',result
        assert len(requests)==(2 if when=='compute' else 1)
        if when=='compute':assert requests[1]['observation_id']!=requests[0]['observation_id']
        assert [c[0] for c in backend.calls]==['open','trajectory','trajectory','close']
        rows=json.loads(next((tmp_path/'plans').glob('*/wrist_00/stability.json')).read_text())
        retry=next(v for v in rows if v['phase']=='retry')
        assert 'a_right_joint3' in retry['reason'] and '0.004' in retry['reason']
        assert sum(v['phase']=='accepted' for v in rows)==(2 if when=='compute' else 1)
    asyncio.run(run())


def test_persistent_capture_motion_is_bounded_and_never_descends_or_closes(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        config=service.profile['wrist_refinement'];config['capture_retries']=1
        capture=service.cameras.capture
        async def moving(args,invocation):
            result=await capture(args,invocation);backend.state['right'][0]+=.004;return result
        service.cameras.capture=moving;service.binding=service._binding()
        planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':planned['outputs']['plan_id']},'unstable')
        assert result['status']=='failed' and 'a_right_joint1' in result['message'],result
        assert len(images)==2 and [c[0] for c in backend.calls]==['open','trajectory']
        rows=json.loads(next((tmp_path/'plans').glob('*/wrist_00/stability.json')).read_text())
        assert len([v for v in rows if v['phase']=='retry'])==2
    asyncio.run(run())


def test_production_alignment_is_required_and_matched_pose_is_sent_to_worker(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        calls=[]
        def aligned(stamp,**limits):
            calls.append(stamp)
            return dict(method='ros_stamp_interpolation',right=[q+.0001 for q in backend.state['right']])
        backend.wrist_joint_state=aligned
        original=service._worker
        async def worker(directory):
            request=json.loads((directory/'request.json').read_text())
            if request['operation']!='refine_wrist':return await original(directory)
            assert request['state']['right']==pytest.approx([q+.0001 for q in backend.state['right']])
            plan=deepcopy(request['reference_plan'])
            return dict(plan=plan,reference_plan=plan,validation={'status':'passed'},converged=True)
        service._worker=worker
        planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':planned['outputs']['plan_id']},'aligned')
        assert result['status']=='succeeded' and len(calls)>=4,result
    asyncio.run(run())


def test_missing_timestamp_alignment_never_falls_back_to_latest_state(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        backend.wrist_joint_state=lambda *args,**kwargs:None
        config=service.profile['wrist_refinement'];config.update(frame_timeout_sec=.1,capture_retries=1)
        service.binding=service._binding();planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':planned['outputs']['plan_id']},'no-sync')
        assert result['status']=='failed' and 'bracketing' in result['message'],result
        assert [c[0] for c in backend.calls]==['open','trajectory']
    asyncio.run(run())


def test_ros_feedback_history_requires_a_complete_same_stamp_joint_vector(monkeypatch):
    from test_grasp_base_feedback import backend
    b,clock=backend(monkeypatch)
    names=b.joints('right')
    b._update('right',(1,0),[(n,0.,0.) for n in names[:3]])
    assert not hasattr(b,'right_joint_history')
    b._update('right',(1,10_000_000),[(n,0.,0.) for n in names])
    clock.now+=.02
    b._update('right',(1,30_000_000),[(n,.002,0.) for n in names])
    result=b.wrist_joint_state(1.02,max_skew_sec=.03,max_span_rad=.003)
    assert result['right']==pytest.approx([.001]*6)
    # Duplicate topic stamp cannot refresh the history or its freshness.
    b._update('right',(1,30_000_000),[(n,.5,0.) for n in names])
    assert len(b.right_joint_history.samples)==2


@pytest.mark.parametrize('persistent',[False,True])
def test_visual_inconsistency_reacquires_without_spending_extra_motion_updates(tmp_path,persistent):
    from hei_agent_bridge.motion import MotionComputeError
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        config=service.profile['wrist_refinement'];before_updates=config['max_updates'];before_tolerance=config['tolerance_m']
        original=service._worker;requests=[]
        async def compute(directory):
            request=json.loads((directory/'request.json').read_text())
            if request['operation']!='refine_wrist':return await original(directory)
            requests.append(request)
            if persistent or len(requests)==1:raise MotionComputeError('Incomplete silhouette','wrist_observation_unavailable')
            plan=deepcopy(request['reference_plan'])
            return dict(plan=plan,reference_plan=plan,validation={'status':'passed'},converged=True)
        service._worker=compute
        planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',dict(plan_id=planned['outputs']['plan_id']),'visual-retry')
        assert result['status']==('failed' if persistent else 'succeeded'),result
        assert len(requests)==(3 if persistent else 2)
        assert len({r['observation_id'] for r in requests})==len(requests)
        assert [c[0] for c in backend.calls]==(['open','trajectory'] if persistent else ['open','trajectory','trajectory','close'])
        assert config['max_updates']==before_updates and config['tolerance_m']==before_tolerance
    asyncio.run(run())


def test_motion_worker_reports_structured_failure_and_never_reuses_old_result(tmp_path):
    from hei_agent_bridge.motion import MotionComputeError
    from test_arm_motion import make_arm_service
    async def run():
        service,_,_=make_arm_service(tmp_path)
        # Bind base implementation, since orchestration fixtures intentionally stub compute.
        from hei_agent_bridge.motion import MotionRuntime
        worker=tmp_path/'compute.py'
        worker.write_text('import json,sys\nfrom pathlib import Path\nPath(sys.argv[1],"result.json").write_text(json.dumps({"error":"partial mask","error_code":"wrist_observation_unavailable"}))\nraise SystemExit(1)\n')
        service.worker_script=str(worker)
        service.profile['persistent_worker'] = False  # This fixture exercises the one-shot protocol.
        directory=tmp_path/'worker';directory.mkdir()
        with pytest.raises(MotionComputeError) as error:await MotionRuntime._worker(service,directory)
        assert error.value.error_code=='wrist_observation_unavailable'
        worker.write_text('raise SystemExit(0)\n')
        with pytest.raises(RuntimeError,match='Motion compute failed'):await MotionRuntime._worker(service,directory)
        assert not (directory/'result.json').exists()
    asyncio.run(run())
