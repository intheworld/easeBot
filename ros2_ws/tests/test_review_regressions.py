"""Review fixes: sensor-derived session state, surface selection, continuous contact."""
import asyncio
import numpy as np
import pytest
from hei_agent_bridge.arm.payload import CarryState
from hei_agent_bridge.grasp.contact import contact_evidence
from hei_agent_bridge.grasp.place_region import estimate_region
from test_grasp_skills import make_service,make_obb_plan,invoke


def test_startup_with_partly_closed_jaw_is_unknown_not_empty(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        backend.state['right_gripper']=-1.5
        result=await make_obb_plan(skills)
        assert result['status']=='failed' and 'hold an object' in result['message']
        assert service.carry.status=='unknown' and not backend.calls
    asyncio.run(run())


def test_open_or_zero_closed_feedback_establishes_initial_empty():
    for position in [-4.,0.,-.0002]:
        c=CarryState();assert c.status=='unknown'
        c.configure_gripper(-4.,-.1/4.5);c.observe({'right_gripper':position})
        c.require_empty();assert c.payload is None


def test_empty_closure_clears_pending_model_and_allows_next_plan(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        original=backend.grasp_gripper
        async def close(invocation,gap,*,close):
            result=await original(invocation,gap,close=close)
            if close:backend.state['right_gripper']=0.
            return result
        backend.grasp_gripper=close
        backend.gripper_contact=lambda _:dict(physical_capture_verified=False,
            contact_state='closed_without_object',torque_nm=.1)
        planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':planned['outputs']['plan_id']},'grasp')
        assert result['status']=='succeeded' and not result['outputs']['grasp_verified']
        assert service.carry.status=='empty' and service.carry.payload is None
        observed=await invoke(skills,'perceive_grasp_target',{'observation_id':'image','target':'cube'},'detect2')
        again=await invoke(skills,'plan_right_grasp_from_object',{'object_id':observed['outputs']['object_id']},'plan2')
        assert again['status']=='succeeded',again
        assert len(backend.calls)==4
    asyncio.run(run())


def test_unverified_closure_retains_unknown_model_instead_of_attaching(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        backend.gripper_contact=lambda _:dict(physical_capture_verified=False,contact_state='feedback_unavailable',torque_nm=0.)
        planned=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':planned['outputs']['plan_id']},'grasp')
        assert result['status']=='succeeded' and not result['outputs']['grasp_verified']
        assert service.carry.status=='unknown' and service.carry.payload is not None
        with pytest.raises(RuntimeError):service.carry.require_empty()
    asyncio.run(run())


def test_sparse_or_reordered_feedback_never_confirms_sustained_contact():
    for times in [[9.65,9.66,9.99],[9.65,9.74,9.73,9.82,9.91,9.99]]:
        r=contact_evidence([(t,-1.5,0.,.2,1.) for t in times],now=10.,closed_gap_m=0.,open_gap_m=.1,gap_per_rad=.1/4.5)
        assert not r['physical_capture_verified'] and r['contact_state']=='feedback_discontinuous'


def test_denser_lower_patch_does_not_replace_upper_support():
    def plane(n,z,offset):
        x,y=np.meshgrid(np.linspace(0,.04,n),np.linspace(0,.04,n))
        return np.column_stack((x.ravel()+offset,y.ravel(),np.full(x.size,z)))
    diag={};r=estimate_region(np.concatenate([plane(20,.4,0),plane(10,.43,.06)]),np.eye(4),'on',diagnostics=diag)
    assert r['T_footprint_region'][2][3]==pytest.approx(.43)
    assert r['T_footprint_region'][0][3]>.06
    assert diag['support_plane']['selection']=='highest_observed_surface'


def test_online_mesh_helpers_do_not_import_offline_mujoco():
    import subprocess,sys,os
    result=subprocess.run([sys.executable,'-c',
        "import sys;from hei_agent_bridge.collision.meshes import stl_components;from hei_agent_bridge.collision import build_scene;assert 'mujoco' not in sys.modules;assert 'hei_agent_bridge.collision.mujoco_scene' not in sys.modules"],
        capture_output=True,text=True,env=os.environ)
    assert result.returncode==0,result.stderr
