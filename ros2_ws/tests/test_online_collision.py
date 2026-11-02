"""Dedicated online collision backend: geometry, policy and offline parity."""
import builtins
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).parents[1]/'src/hei_agent_bridge/src'))
pytest.importorskip('fcl')
from hei_agent_bridge.collision.fcl_scene import FCLScene,robot_model
from hei_agent_bridge.grasp.validation import CollisionScreen,set_pose,relative_tcp,build_scene
from hei_agent_bridge.grasp.kinematics import RightArmFK

URDF=Path(__file__).parents[1]/'src/hei_description/urdf/hei_body.urdf'
SCENE=dict(table_top_z_m=-.1,left_joint_rad=[0,-.5,-.5,0,0,0],lift_joint_m=0.,left_finger_opening_m=.04,clearance_m=0.,scene_confirmed=True)
Q=np.array([0,-.5,-.5,0,0,0])


def test_online_scene_never_imports_simulator(monkeypatch):
    real=builtins.__import__
    def guarded(name,*a,**kw):
        if name.startswith('mujoco'):raise AssertionError('Online simulator dependency')
        return real(name,*a,**kw)
    monkeypatch.setattr(builtins,'__import__',guarded)
    model=FCLScene(URDF,{},SCENE)
    set_pose(model,model.data,Q,-3.)
    np.testing.assert_allclose(relative_tcp(model,model.data),RightArmFK(URDF)(Q),atol=1e-10)


def test_scene_motion_and_obstacles_are_not_cached():
    assert robot_model(URDF) is robot_model(URDF)
    a=FCLScene(URDF,{},SCENE);b=FCLScene(URDF,{},dict(SCENE,lift_joint_m=-.1))
    a.set_pose(Q,-2);b.set_pose(Q+.01,-2)
    assert not np.allclose(a.data.xpos,b.data.xpos)
    assert a.data is not b.data and a.objects[0] is not b.objects[0]


def test_colliding_obstacle_detected_and_classified():
    model=FCLScene(URDF,{},SCENE);model.set_pose(Q,-3)
    t=model.transforms['a_right_tcp'].copy()
    obstacle={'id':'test_coverage_unknown_0','size_m':[.2,.2,.2],'T_footprint_object':t.tolist()}
    model=FCLScene(URDF,{'carry':{'obstacles':[obstacle]}},SCENE);model.set_pose(Q,-3)
    screen=CollisionScreen(model,model.data,SCENE);screen.check('start',0)
    assert any(c['constraint_kind']=='unobserved_space' for c in screen.collisions.values())


def test_payload_contact_does_not_hide_arm_or_world_collision():
    plan={'carry':{'status':'attached','payload':{'T_tcp_object':np.eye(4).tolist(),'size_m':[.4,.4,.4]}}}
    model=FCLScene(URDF,plan,SCENE);model.set_pose(Q,-3)
    screen=CollisionScreen(model,model.data,SCENE);screen.check('start',0)
    collisions=list(screen.collisions.values())
    assert any('a_right_payload' in c['bodies'] for c in collisions)
    assert not any('a_right_payload' in c['bodies'] and set(c['bodies'])&screen.right_fingers for c in collisions)


def test_urdf_fk_and_collision_decisions_against_offline_simulator():
    pytest.importorskip('mujoco')
    online=FCLScene(URDF,{},SCENE);offline,data=build_scene(URDF,{},SCENE,targets=[])
    rng=np.random.default_rng(37)
    missed=[]
    for index in range(40):
        q=rng.uniform([-.2,-2,-2,-1,-1,-2],[1.4,.5,.5,1.4,1,2]);grip=float(rng.uniform(-4.5,0))
        set_pose(online,online.data,q,grip);set_pose(offline,data,q,grip)
        np.testing.assert_allclose(relative_tcp(online,online.data),relative_tcp(offline,data),atol=1e-9)
        a=CollisionScreen(online,online.data,SCENE);b=CollisionScreen(offline,data,SCENE)
        a.check('pose',1);b.check('pose',1)
        # Compare body-pair coverage, excluding numerical grazing (< 0.1 mm).
        found={frozenset(c['bodies']) for c in a.collisions.values()}
        for c in b.collisions.values():
            if c['distance_m']<-.0001 and 'validation_table' not in c['geoms'] and frozenset(c['bodies']) not in found:
                missed.append((index,c['bodies'],c['distance_m']))
        assert not b.collisions or a.collisions
    assert not missed,missed

@pytest.mark.parametrize('second_passes',[False,True])
def test_release_preflight_checks_alternatives_and_never_certifies_plan(monkeypatch,second_passes):
    from types import SimpleNamespace
    from hei_agent_bridge.arm import place
    fk=RightArmFK(URDF)
    planner=SimpleNamespace(fk=fk,choose=lambda goal,seed,candidates:np.asarray(candidates[0]))
    request={'urdf':str(URDF),'state':{'right':Q.tolist(),'right_gripper':-2,'lift':0},
        'profile':{'gap_per_motor_rad_m':.1/4.5,'release_clearance_m':.01,'joint_speed_rad_s':.12},
        'arguments':{'release_gap_m':.09},'_collision_scene':SCENE}
    carry={'status':'attached','payload':{'id':'held','T_tcp_object':np.eye(4).tolist(),'size_m':[.02,.02,.02]},'obstacles':[]}
    calls=[]
    def validate(plan,*args,**kwargs):
        calls.append(plan)
        return {'status':'passed' if second_passes and len(calls)>=2 else 'failed','collision_pairs':[]}
    monkeypatch.setattr(place,'validate_arm_plan',validate)
    candidates=[Q,Q.copy(),Q+.01]
    if second_passes:
        assert place.release_preflight(request,planner,fk(Q),candidates,carry,.01) is None
    else:
        with pytest.raises(place.PlacementPreflightError) as error:
            place.release_preflight(request,planner,fk(Q),candidates,carry,.01)
        assert error.value.result['validation']['status']=='failed'
        assert error.value.result['validation']['unchecked_stages']==['transfer','retreat']
    assert len(calls)==(3 if second_passes else 2)
    assert all(p['preflight_only'] for p in calls)


def test_passed_plan_checks_full_trajectory_and_release():
    from hei_agent_bridge.arm.planner import validate_arm_plan,trajectory_samples
    fk=RightArmFK(URDF);end=Q+np.array([.02,0,0,0,0,0])
    plan=dict(schema='hei.arm_plan/v1',urdf_sha256=fk.sha256,base_frame=fk.base_frame,tool_frame=fk.tool_frame,
        start_joint_rad=Q.tolist(),waypoints_rad=[end.tolist()],start_gripper_rad=-2.,gripper_end_rad=-3.,
        joint_speed_rad_s=.12,T_start=fk(Q).tolist(),T_goal=fk(end).tolist(),nominal_duration_sec=2.)
    result=validate_arm_plan(plan,URDF,SCENE)
    assert result['status']=='passed',result
    assert result['collision_backend']=='fcl'
    expected=len(list(trajectory_samples(Q,[end],.12)))+51
    assert result['samples']==expected
    assert not result['stopped_on_collision']


def test_positive_clearance_and_exclusion_depth_are_preserved():
    # Robot meshes are compared at a known separated pose. Two explicit boxes
    # near the TCP exercise distance queries and bounded contact exceptions.
    base=FCLScene(URDF,{},SCENE);base.set_pose(Q,-3.)
    t=base.transforms['a_right_tcp'].copy()
    carry={'status':'attached','payload':{'T_tcp_object':np.eye(4).tolist(),'size_m':[.01,.01,.01]},'obstacles':[]}
    obstacle=t.copy();obstacle[:3,3]+=t[:3,0]*.012
    carry['obstacles']=[{'id':'near','T_footprint_object':obstacle.tolist(),'size_m':[.01,.01,.01]}]
    scene=dict(SCENE,payload_padding_m=0.,clearance_m=.003)
    model=FCLScene(URDF,{'carry':carry},scene);model.set_pose(Q,-3.)
    screen=CollisionScreen(model,model.data,scene);screen.check('pose',1.)
    pair=[c for c in screen.collisions.values() if set(c['bodies'])=={'a_right_payload','placed_near'}]
    assert pair and pair[0]['distance_m']==pytest.approx(.002,abs=1e-6)


def test_reused_collision_context_rejects_changed_scene():
    from hei_agent_bridge.arm.planner import validate_arm_plan
    from hei_agent_bridge.collision import build_scene as online_scene
    fk=RightArmFK(URDF)
    plan=dict(schema='hei.arm_plan/v1',urdf_sha256=fk.sha256,base_frame=fk.base_frame,tool_frame=fk.tool_frame,
        start_joint_rad=Q.tolist(),waypoints_rad=[Q.tolist()],start_gripper_rad=-2.,
        joint_speed_rad_s=.12,T_start=fk(Q).tolist(),T_goal=fk(Q).tolist(),nominal_duration_sec=0.)
    context=online_scene(URDF,plan,SCENE)
    assert validate_arm_plan(plan,URDF,SCENE,_collision_scene=context)['status']=='passed'
    with pytest.raises(ValueError,match='context differs'):
        validate_arm_plan(plan,URDF,dict(SCENE,table_top_z_m=.2),_collision_scene=context)


@pytest.mark.parametrize('lift',[4.24989548356349e-6,.01,-.81])
def test_observed_lift_is_not_clamped_or_range_rejected(lift):
    model=FCLScene(URDF,{},dict(SCENE,lift_joint_m=lift))
    zero=FCLScene(URDF,{},dict(SCENE,lift_joint_m=0.))
    assert model.values['lift_joint']==lift
    delta=model.transforms['lift_carriage_link'][:3,3]-zero.transforms['lift_carriage_link'][:3,3]
    assert np.linalg.norm(delta)==pytest.approx(abs(lift),abs=1e-12)


def test_observed_left_arm_range_is_not_a_right_arm_goal_constraint():
    q=[.5,.02,-.5,0,0,0]
    model=FCLScene(URDF,{},dict(SCENE,left_joint_rad=q))
    assert model.values['b_left_joint2']==.02


def test_nonfinite_scene_feedback_still_rejected():
    with pytest.raises(ValueError):
        FCLScene(URDF,{},dict(SCENE,lift_joint_m=float('nan')))


def test_pose_contact_reuse_is_exact_and_invalidated_by_geometry_updates():
    model=FCLScene(URDF,{},SCENE)
    model.set_pose(Q,-3.);queries=model.query_count
    model.set_pose(Q.copy(),-3.)
    assert model.query_count==queries
    model.update_transforms(Q+.001,-3.)
    model.set_pose(Q,-3.)
    assert model.query_count>queries
    queries=model.query_count
    model.contacts(np.empty((0,2),int))
    model.set_pose(Q,-3.)
    assert model.query_count>queries
