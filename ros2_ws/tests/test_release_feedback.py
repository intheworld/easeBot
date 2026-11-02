"""Release hold uses endpoint evidence; partial results never imply full success."""
import asyncio
import json
from pathlib import Path
import pytest
from test_arm_transport import service_with_place_worker,attach,placement_args,call


async def prepared(tmp_path):
    service,skills,backend=service_with_place_worker(tmp_path);obj=attach(service,backend)
    planned=await call(skills,'plan_place_object',placement_args(service,backend,obj),'plan')
    assert planned['status']=='succeeded',planned
    return service,skills,backend,planned['outputs']['plan_id']


def test_small_joint_settling_during_opening_does_not_abort_valid_tcp(tmp_path):
    async def run():
        service,skills,backend,id=await prepared(tmp_path);normal=backend.gripper
        async def opening(*args):
            backend.state['right'][5]+=.004  # Exceeds the removed .003 rad gate.
            await asyncio.sleep(.12)
            return await normal(*args)
        backend.gripper=opening
        result=await call(skills,'execute_place_plan',{'plan_id':id},'execute')
        assert result['status']=='succeeded',result
        assert result['outputs']['release_confirmed'] and result['outputs']['retracted']
        rows=json.loads((service.directory/id/'release_feedback.json').read_text())
        assert any(abs(r['right_joint_delta_rad'][5])>.003 for r in rows)
        assert all(r['position_error_m']<=.005 for r in rows)
    asyncio.run(run())


@pytest.mark.parametrize('problem',['tcp_drift','base_moved','opening_exception','wrong_gap','retreat'])
def test_partial_stages_and_precise_diagnostics_are_preserved_on_failure(tmp_path,problem):
    async def run():
        service,skills,backend,id=await prepared(tmp_path);normal=backend.gripper
        async def opening(*args):
            if problem=='tcp_drift':backend.state['right'][1]+=.1
            if problem=='base_moved':backend.state['wheels'][0]+=.03
            if problem in ('tcp_drift','base_moved'):await asyncio.sleep(.12)
            if problem=='opening_exception':raise RuntimeError('opening action failed')
            value=await normal(*args)
            if problem=='wrong_gap':backend.state['right_gripper']=0.
            return value
        backend.gripper=opening
        if problem=='retreat':
            async def fail(directory):return {'validation':{'status':'failed'}}
            service._worker=fail
        result=await call(skills,'execute_place_plan',{'plan_id':id},'execute')
        assert result['status']=='failed',result
        outputs=result['outputs'];assert outputs['release_started']
        assert outputs['placed']==(problem=='retreat')
        assert outputs['release_confirmed']==(problem=='retreat')
        assert not outputs['retracted'] and not outputs['placement_verified']
        assert outputs['failed_stage']==('retreat' if problem=='retreat' else 'release')
        assert Path(outputs['detail_ref']).is_file()
        if problem=='retreat':
            assert 'Object release confirmed; retreat incomplete' in result['message']
            assert service.carry.status=='empty'
        else:
            assert service.carry.status=='unknown'
            assert len([c for c in backend.calls if c[0]=='arm'])==1
        if problem in ('tcp_drift','base_moved'):
            rows=json.loads((service.directory/id/'release_feedback.json').read_text())
            assert rows[-1]['status']=='failed' and rows[-1]['reason']
    asyncio.run(run())


def test_opening_confirmed_but_pose_not_settled_keeps_partial_evidence(tmp_path):
    async def run():
        service,skills,backend,id=await prepared(tmp_path);original=service.state;normal=backend.gripper
        opened=False
        async def opening(*args):
            nonlocal opened
            result=await normal(*args);opened=True
            return result
        def state(*,at_rest=True):
            if opened and at_rest:raise RuntimeError('Grasp scene is moving: right')
            return original(at_rest=at_rest)
        backend.gripper=opening;service.state=state
        result=await call(skills,'execute_place_plan',{'plan_id':id},'execute')
        assert result['status']=='failed',result
        assert result['outputs']['release_confirmed'] and not result['outputs']['placed']
        assert not result['outputs']['retracted']
        assert 'opening confirmed' in result['message'].lower()
        assert service.carry.status=='unknown'
    asyncio.run(run())
