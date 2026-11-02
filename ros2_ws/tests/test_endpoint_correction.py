"""Offline recovery contracts: no ROS, no physical motion."""
import asyncio
import json
import numpy as np
import pytest
from test_arm_motion import make_arm_service
from hei_agent_bridge.arm.correction import correct_endpoint


@pytest.mark.parametrize('mode',['converge','collision','stalled','cancel','drift'])
def test_endpoint_correction_is_bounded_and_validated(tmp_path,mode):
    async def run():
        service,_,backend=make_arm_service(tmp_path)
        q=np.array(backend.state['right']);q[0]+=.06
        target=service.fk(q)
        plan=dict(T_goal=target.tolist(),waypoints_rad=[q.tolist()])
        requests=[]
        async def worker(path):
            request=json.loads((path/'request.json').read_text());requests.append(request)
            if mode=='cancel':raise asyncio.CancelledError()
            if mode=='drift':backend.state['right'][0]+=.04
            return dict(validation={'status':'failed' if mode=='collision' else 'passed'},
                        plan=dict(waypoints_rad=[q.tolist()],joint_speed_rad_s=.12))
        service._worker=worker
        if mode=='stalled':
            async def stuck(*args):backend.calls.append(('arm',))
            backend.arm_trajectory=stuck
        if mode=='converge':
            state,verified=await correct_endpoint(service,plan,'test',tmp_path/'correction')
            service.verify_endpoint(verified,state)
            assert len(backend.calls)==1
            assert requests[0]['arguments']['x_m']==target[0,3]
        else:
            with pytest.raises(asyncio.CancelledError if mode=='cancel' else RuntimeError):
                await correct_endpoint(service,plan,'test',tmp_path/'correction')
            assert len(backend.calls)==(1 if mode=='stalled' else 0)
        records=json.loads((tmp_path/'correction/feedback.json').read_text())
        assert len(records[0]['actual_minus_target_xyz_m'])==3
        assert len(requests)==1
    asyncio.run(run())


def test_reached_endpoint_does_not_replan(tmp_path):
    async def run():
        service,_,backend=make_arm_service(tmp_path)
        plan=dict(T_goal=service.fk(backend.state['right']).tolist(),waypoints_rad=[backend.state['right']])
        async def unexpected(path):raise AssertionError('unnecessary correction')
        service._worker=unexpected
        await correct_endpoint(service,plan,'test',tmp_path/'correction')
        assert not backend.calls
    asyncio.run(run())


def test_second_correction_and_hard_attempt_budget(tmp_path):
    async def run(converge):
        service,_,backend=make_arm_service(tmp_path/str(converge))
        start=np.array(backend.state['right']);goal=start.copy();goal[0]+=.12
        plan=dict(T_goal=service.fk(goal).tolist(),waypoints_rad=[goal.tolist()])
        count=0
        async def worker(path):
            return dict(validation={'status':'passed'},plan=dict(waypoints_rad=[goal.tolist()],joint_speed_rad_s=.12))
        async def move(*args):
            nonlocal count
            count+=1
            backend.state['right']=(goal if converge and count==2 else start+np.array([.03*count,0,0,0,0,0])).tolist()
        service._worker=worker;backend.arm_trajectory=move
        if converge:
            await correct_endpoint(service,plan,'test',tmp_path/str(converge)/'correction')
        else:
            with pytest.raises(RuntimeError,match='two corrections'):
                await correct_endpoint(service,plan,'test',tmp_path/str(converge)/'correction')
        assert count==2
    asyncio.run(run(True));asyncio.run(run(False))


def test_place_corrects_before_opening(tmp_path):
    from test_arm_transport import service_with_place_worker,attach,placement_args,call
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path)
        obj=attach(service,backend)
        result=await call(skills,'plan_place_object',placement_args(service,backend,obj),'plan')
        assert result['status']=='succeeded',result
        normal=backend.arm_trajectory;count=0
        async def lag_once(*args):
            nonlocal count
            await normal(*args);count+=1
            if count==1:backend.state['right'][0]-=.05
        backend.arm_trajectory=lag_once
        result=await call(skills,'execute_place_plan',dict(plan_id=result['outputs']['plan_id']),'execute')
        assert result['status']=='succeeded',result
        assert [x[0] for x in backend.calls]==['arm','arm','release','arm']
    asyncio.run(run())


@pytest.mark.parametrize('error',[None,'Stale joint feedback','Grasp scene is moving: right'])
def test_endpoint_waits_for_transient_right_velocity_only(tmp_path,error):
    async def run():
        service,_,backend=make_arm_service(tmp_path)
        plan=dict(T_goal=service.fk(backend.state['right']).tolist(),waypoints_rad=[backend.state['right']])
        original=service.state;reads=0
        def state(*,at_rest=True):
            nonlocal reads
            if at_rest:
                reads+=1
                if error is not None:raise RuntimeError(error)
                if reads<3:raise RuntimeError('Grasp scene is moving: right')
            return original(at_rest=at_rest)
        service.state=state
        if error is None:
            await correct_endpoint(service,plan,'test',tmp_path/'correction')
            assert reads==3
        else:
            with pytest.raises(RuntimeError,match='did not settle' if 'moving' in error else 'Stale'):
                await correct_endpoint(service,plan,'test',tmp_path/'correction')
            if 'Stale' in error:assert reads==1
        assert not backend.calls
    asyncio.run(run())
