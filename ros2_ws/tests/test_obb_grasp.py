"""GaP-inspired geometry boundary, candidate planning and no-motion perception."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.grasp.obb import (visible_obb, segmented_cloud,gripper_geometry,
                                        top_down_candidates,support_geometry)
from hei_agent_bridge.grasp.validation import trajectory_samples
from test_grasp_skills import make_service,invoke

URDF=ROOT/'ros2_ws/src/hei_description/urdf/hei_body.urdf'
PROFILE=json.loads((ROOT/'ros2_ws/src/hei_agent_bridge/config/right_grasp.json').read_text())


def box_points(yaw=.6):
    values=np.linspace(-.5,.5,11)
    points=np.array(np.meshgrid(values,values,values)).reshape(3,-1).T*np.array([.06,.03,.04])
    c,s=np.cos(yaw),np.sin(yaw)
    rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    return points@rotation.T+[.4,.1,.2],rotation


def test_obb_pose_size_and_gravity():
    points,rotation=box_points()
    obb=visible_obb(points,[0,0,1])
    np.testing.assert_allclose(obb['center_m'],[.4,.1,.2],atol=1e-6)
    np.testing.assert_allclose(obb['size_m'],[.06,.03,.04],atol=1e-6)
    assert abs(np.dot(np.asarray(obb['rotation'])[:,0],rotation[:,0]))> .999
    assert obb['geometry_scope']=='visible_surface_only'
    tilted=np.array([[1,0,0],[0,0,-1],[0,1,0]])
    other=visible_obb(points@tilted.T,tilted[:,2])
    np.testing.assert_allclose(other['size_m'],obb['size_m'],atol=1e-6)


def test_urdf_finger_envelope_and_topdown_candidates():
    geometry=gripper_geometry(URDF,PROFILE)
    assert 0 <= geometry['finger_forward_m'] < .1
    assert abs(np.dot(geometry['closing_axis'],[0,1,0]))> .999
    points,rotation=box_points()
    obb=visible_obb(points,[0,0,1])
    candidates=top_down_candidates(obb,geometry,PROFILE,.1)
    assert len(candidates)==8
    for candidate in candidates:
        goal,pre=np.array(candidate['T_grasp']),np.array(candidate['T_pregrasp'])
        np.testing.assert_allclose(goal[:3,0],[0,0,-1],atol=1e-6)
        np.testing.assert_allclose(goal[:3,:3],pre[:3,:3])
        np.testing.assert_allclose(pre[:3,3]-goal[:3,3],[0,0,.08])
        assert np.linalg.det(goal[:3,:3])==pytest.approx(1.)
    assert candidates[0]['jaw_width_m']==pytest.approx(.03,abs=1e-6)
    with pytest.raises(ValueError,match='too low'):
        top_down_candidates(obb,geometry,PROFILE,.3)
    with pytest.raises(ValueError,match='opening'):
        top_down_candidates(obb,{**geometry,'open_gap_m':.01},PROFILE,.1)


def test_topdown_has_no_implicit_base_zero_floor():
    points,_=box_points()
    obb=visible_obb(points,[0,0,1]);obb['center_m'][2]=-.15
    geometry=gripper_geometry(URDF,PROFILE)
    candidates=top_down_candidates(obb,geometry,PROFILE,-.3)
    assert all(c['T_grasp'][2][3]<0 for c in candidates)
    with pytest.raises(ValueError,match='explicitly configured'):
        top_down_candidates(obb,geometry,{**PROFILE,'min_base_z_m':0.},-.3)


def test_segmented_cloud_uses_only_matching_mask_and_depth():
    mask=np.zeros((1,12,12));mask[:,2:10,2:10]=1
    result=SimpleNamespace(boxes=SimpleNamespace(cls=np.array([0]),conf=np.array([.9])),
                           masks=SimpleNamespace(data=mask),names={0:'red cube'})
    depth=np.ones((12,12));depth[3,3]=np.nan
    k=np.array([[100,0,6],[0,100,6],[0,0,1]])
    score,points,selected=segmented_cloud([result],depth,k,'red cube',np.eye(4))
    assert score==.9 and points.shape==(63,3)
    assert np.all(points[:,2]==1)
    with pytest.raises(ValueError,match='No segmented'):
        segmented_cloud([result],depth,k,'banana',np.eye(4))


@pytest.mark.parametrize('case,reason', [
    ('empty','no matching detections'),
    ('no_mask','no segmentation masks'),
    ('no_depth','fewer than 30 valid depth pixels'),
])
def test_segmentation_failure_diagnostics(case,reason):
    result=SimpleNamespace(boxes=SimpleNamespace(cls=np.array([0]),conf=np.array([.9])),
                           masks=SimpleNamespace(data=np.ones((1,12,12))),names={0:'cube'})
    depth=np.ones((12,12))
    if case=='empty':result.boxes.cls=result.boxes.conf=np.array([])
    if case=='no_mask':result.masks=None
    if case=='no_depth':depth[:]=0
    diagnostics={}
    with pytest.raises(ValueError,match=reason):
        segmented_cloud([result],depth,np.eye(3),'cube',np.eye(4),diagnostics)
    assert diagnostics['matching_detections']==(0 if case=='empty' else 1)
    if case=='no_depth':assert diagnostics['objects'][0]['valid_depth_pixels']==0


def test_worker_tries_symmetric_candidate_after_collision(tmp_path,monkeypatch):
    from hei_agent_bridge.grasp import worker
    state={'right':[.1]*6,'right_gripper':0.,'left':[0.]*6,'left_gripper':0.,'lift':0.}
    up,support=support_geometry(URDF,state,PROFILE['scene'])
    points,_=box_points(0.)
    obb=visible_obb(points,up)
    obb['center_m']=(np.array([.4,.1,0])+up*(support+.05)).tolist()
    obb.update(target='cube',confidence=.9)
    monkeypatch.setattr(worker.RightArmPlanner,'solve',lambda *args:np.full(6,.1))
    monkeypatch.setattr(worker.RightArmPlanner,'cartesian_path',lambda *args:[np.full(6,.2)])
    calls=[]
    def validate(plan,*args):
        calls.append(plan)
        return ({'status':'passed' if len(calls)==2 else 'failed','errors':[],
                 'collision_pairs':[] if len(calls)==2 else [{'bodies':['arm','table']}]},None,None)
    monkeypatch.setattr(worker,'validate_plan',validate)
    result=worker.object_plan({'profile':PROFILE,'state':state,'object':obb,'urdf':str(URDF),'directory':str(tmp_path)})
    assert result['validation']['status']=='passed'
    assert result['plan']['candidate_index']==1 and result['plan']['schema_version']==3
    assert len(json.loads((tmp_path/'candidates.json').read_text()))==2
    phases={phase for _,phase,_,_ in trajectory_samples(result['plan'])}
    assert 'retreat' not in phases and 'return' not in phases


def test_bad_starting_gripper_is_reported_before_any_ik(tmp_path,monkeypatch):
    from hei_agent_bridge.grasp import worker
    def unexpected(*args,**kwargs):
        pytest.fail('Invalid starting gripper must be rejected before IK')
    monkeypatch.setattr(worker,'RightArmPlanner',unexpected)
    with pytest.raises(ValueError,match='Invalid starting right gripper: Gripper feedback 0.0597'):
        worker.object_plan({'profile':PROFILE,'state':{'right_gripper':.0597}})


def test_perception_obb_plan_and_stop_after_close(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        detected=await invoke(skills,'perceive_grasp_target',{'observation_id':'image','target':'red cube'},'detect')
        assert detected['status']=='succeeded',detected
        assert not backend.calls
        object_id=detected['outputs']['object_id']
        assert detected['outputs']['geometry_scope']=='visible_surface_only'
        plan=await invoke(skills,'plan_right_grasp_from_object',{'object_id':object_id},'plan')
        assert plan['status']=='succeeded',plan
        request=json.loads((service.directory/plan['outputs']['plan_id']/'request.json').read_text())
        assert request['object']['target']=='red cube'
        result=await invoke(skills,'execute_right_grasp',{'plan_id':plan['outputs']['plan_id']},'execute')
        assert result['status']=='succeeded',result
        assert [c[0] for c in backend.calls]==['open','trajectory','trajectory','close']
        assert result['outputs']['grasp_verified'] is False
    asyncio.run(run())


@pytest.mark.parametrize('cause',['expired','scene','binding','invalid_id'])
def test_object_plan_rejects_stale_or_changed_inputs(tmp_path,cause):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        detected=await invoke(skills,'perceive_grasp_target',{'observation_id':'image','target':'cube'},'detect')
        object_id=detected['outputs']['object_id']
        if cause=='expired':service.objects[object_id]['created']-=61
        if cause=='scene':backend.state['lift']+=.02
        if cause=='binding':Path(service.profile['model_path']).write_bytes(b'changed')
        if cause=='invalid_id':object_id='../object'
        result=await invoke(skills,'plan_right_grasp_from_object',{'object_id':object_id},'reject')
        assert result['status']=='failed',result
        assert not backend.calls
    asyncio.run(run())


def test_examples_validate_against_actual_skill_manifest(tmp_path):
    sys.path.insert(0,str(ROOT/'agent/src'))
    from hei_agent.catalog import SkillCatalog
    from hei_agent.graph import Graph
    from hei_agent.planner import validate_plan
    _,skills,_,_=make_service(tmp_path)
    manifest=skills.manifest()
    # Camera tools are normally registered by SkillDevice, outside this fixture.
    manifest['skills'].append({'name':'get_camera_observation','description':'camera','arguments':{'camera':{'type':'string'}},
        'outputs':{'observation_id':{'type':'string'}},'outcomes':['success','failure'],'motion':False,'version':'1','timeout_sec':2})
    from hei_agent_bridge.skills import digest
    manifest.pop('revision');manifest['revision']=digest(manifest)
    catalog=SkillCatalog.from_manifest(manifest)
    for name in ('right_object_obb.json','right_obb_grasp_plan.json','right_obb_grasp_execute.json'):
        validate_plan(Graph.load(ROOT/'agent/src/hei_agent/examples'/name),{},catalog)


@pytest.mark.parametrize('operation',[None,'legacy','unknown'])
def test_worker_rejects_retired_operations_before_loading_images(operation):
    from hei_agent_bridge.grasp.worker import compute
    with pytest.raises(ValueError,match='Unsupported grasp operation'):
        compute({'operation':operation})


def test_retired_skill_is_not_published(tmp_path):
    _,skills,_,_=make_service(tmp_path)
    names={s['name'] for s in skills.manifest()['skills']}
    assert 'plan_right_grasp' not in names
    assert {'perceive_grasp_target','plan_right_grasp_from_object','execute_right_grasp'}<=names


@pytest.mark.parametrize('plan',[{}, {'schema_version':2,'strategy':'top_down_direct_ik'}])
def test_validation_rejects_retired_plans(plan):
    from hei_agent_bridge.grasp.validation import validate_plan
    with pytest.raises(ValueError,match='Unsupported grasp plan'):
        validate_plan(plan,URDF,PROFILE['scene'])
