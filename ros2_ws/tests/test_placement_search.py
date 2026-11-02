"""Candidate diversity and geometry invariants; no robot commands."""
from itertools import islice
import numpy as np
import pytest
from hei_agent_bridge.arm.spatial import supported_centers
from hei_agent_bridge.arm.place import region_place_candidates, placement_targets
from hei_agent_bridge.collision.fcl_scene import collision_pairs
from test_place_regions import declared, plan_args, region_geometry
from test_arm_transport import make_arm_service, attach, URDF


def test_diverse_samples_cover_supported_region_early():
    region={'spatial_model':{'schema':'hei.container_space/v2','coverage':{
        'cells':np.ones((20,20),int).tolist(),'resolution_m':.01,
        'origin_xy_m':[-.1,-.1],'shape_xy':[20,20]}}}
    points=np.array(list(islice(supported_centers(region,np.array([.01,.01]),
                                                 np.array([.08,.08]),diverse=True),9)))
    assert np.ptp(points,axis=0).min()>.15
    assert len(np.unique(points,axis=0))==9


def test_yaw_candidates_preserve_object_position_and_gravity(tmp_path):
    service,_,backend=make_arm_service(tmp_path);obj=attach(service,backend)
    args=declared(service,backend,obj,'inside')
    region=region_geometry(np.eye(4),*[args[k] for k in ('x_m','y_m','support_z_m','size_x_m',
        'size_y_m','yaw_rad','relation','wall_height_m','boundary_thickness_m')])
    region['id']='test'
    request=dict(urdf=str(URDF),profile=service.profile,state=backend.state,carry=service.carry.snapshot(),
                 arguments={**plan_args('test'),'region':region,'path_mode':'linear','max_tilt_deg':0.})
    candidates=list(region_place_candidates(request))
    assert len(candidates)<=32
    assert len({c['arguments']['_placement_yaw_rad'] for c in candidates[:6]})==6
    centers=[]
    for c in candidates[:6]:
        goal=placement_targets(c)[3]
        mount=np.asarray(request['carry']['payload']['T_tcp_object'])
        centers.append((goal@mount)[:3,3])
        assert c['arguments']['release_gap_m']==request['arguments']['release_gap_m']
    np.testing.assert_allclose(centers,np.broadcast_to(centers[0],(6,3)),atol=1e-9)


def test_collision_pair_cache_tracks_topology_and_is_read_only():
    args=(('base','arm','wall'),('arm',),(('base','base'),('arm','arm')),(), 'base')
    a=collision_pairs(*args);b=collision_pairs(*args)
    assert a is b and not a.flags.writeable
    c=collision_pairs(('base','arm','wall','other'),*args[1:])
    assert len(c)>len(a)


def test_region_ik_is_batched_and_does_not_certify_collision(monkeypatch):
    from types import SimpleNamespace
    from hei_agent_bridge.arm import place
    from hei_agent_bridge.grasp import planner_backend
    calls=[];seen=[]
    planner=SimpleNamespace(backend='curobo',proposals=lambda goals,seed:
                            (calls.append(len(goals)) or np.zeros((len(goals),8,6))))
    monkeypatch.setattr(planner_backend,'get_planner',lambda *args:planner)
    request={'arguments':{'region':{'id':'r'}},'profile':{},'urdf':'unused','state':{'right':[0.]*6}}
    candidates=[{**request,'arguments':{**request['arguments'],'x_m':i,'y_m':0.}} for i in (0,1)]
    monkeypatch.setattr(place,'region_place_candidates',lambda r:iter(candidates))
    monkeypatch.setattr(place,'placement_targets',lambda r:(None,None,None,np.eye(4),0.))
    def plan(r):
        assert r['_release_solutions'].shape==(8,6)
        seen.append(r['arguments']['x_m'])
        return {'index':r['arguments']['x_m']}
    monkeypatch.setattr(place,'make_place_plan',plan)
    monkeypatch.setattr(place,'validate_place_plan',lambda p,*a,**kw:{'status':'passed' if p['index']==1 else 'failed'})
    result=place.compute_region_place(request,{})
    assert calls==[2] and seen==[0,1]
    assert result['plan']['index']==1 and result['validation']['status']=='passed'


def test_failed_search_reports_furthest_stage_and_retains_other_failures(monkeypatch):
    from types import SimpleNamespace
    from hei_agent_bridge.arm import place
    from hei_agent_bridge.grasp import planner_backend
    request={'arguments':{'region':{'id':'r'}},'profile':{},'urdf':'unused','state':{'right':[0.]*6}}
    candidates=[{**request,'arguments':{**request['arguments'],'x_m':i,'y_m':0.}} for i in (0,1)]
    monkeypatch.setattr(planner_backend,'get_planner',lambda *args:SimpleNamespace(backend='scipy'))
    monkeypatch.setattr(place,'region_place_candidates',lambda r:iter(candidates))
    early={'status':'failed','stages':{'opening':{'status':'failed'}}}
    late={'status':'failed','stages':{'opening':{'status':'passed'},'transfer':{'status':'failed'}}}
    def plan(r):
        if r['arguments']['x_m']==0:
            raise place.PlacementPreflightError({'plan':{'index':0},'validation':early})
        return {'index':1}
    monkeypatch.setattr(place,'make_place_plan',plan)
    monkeypatch.setattr(place,'validate_place_plan',lambda *args,**kwargs:late)
    result=place.compute_region_place(request,{})
    assert result['plan']['index']==1 and result['validation']==late
    assert [a['validation'] for a in result['placement_attempts']]==[early,late]


def test_batched_release_rejection_shares_scene_but_never_certifies_a_pass(monkeypatch):
    from types import SimpleNamespace
    from hei_agent_bridge.arm import place
    from hei_agent_bridge import collision
    from hei_agent_bridge.collision import cuda_batch
    from hei_agent_bridge.grasp.kinematics import RightArmFK
    fk=RightArmFK(URDF);q=np.array([.3,-1.,-1.,.5,.1,.2])
    request={'urdf':str(URDF),'state':{'right':q.tolist()},
             'profile':{'gap_per_motor_rad_m':.1/4.5,'joint_speed_rad_s':.12},
             'arguments':{'release_gap_m':.05},
             'carry':{'status':'attached','payload':{'id':'held'},'obstacles':[{'id':'tray'}]}}
    candidates=[{**request,'_release_solutions':np.array([q,q.copy(),q+.01]),'index':i} for i in range(2)]
    planner=SimpleNamespace(fk=fk,choose=lambda goal,seed,qs:np.array(qs[0]))
    monkeypatch.setattr(place,'placement_targets',lambda c:(None,None,None,fk(q),0.))
    builds=[];rows=[]
    monkeypatch.setattr(collision,'build_scene',lambda *args:
                        (builds.append(args) or (SimpleNamespace(collision_accelerator='test'),None)))
    def posed(model,data,samples,**kw):
        assert kw['first_cpu'] is False
        for row in samples:rows.append(row);yield row
    monkeypatch.setattr(cuda_batch,'posed_samples',posed)
    calls=[]
    def validate(plan,*args,**kw):
        assert plan['carry']['payload'] is None and plan['carry']['obstacles']==[{'id':'tray'}]
        # First owner fails both configs. Second owner passes one, which does
        # NOT skip its subsequent full placement validation.
        calls.append(plan)
        return {'status':'passed' if len(calls)==3 else 'failed','collision_pairs':[]}
    monkeypatch.setattr(place,'validate_arm_plan',validate)
    report=place.batch_release_screen(request,candidates,planner,{})
    assert len(builds)==1 and len(rows)==4
    assert report['configurations']==4 and report['rejected_candidates']==1
    assert candidates[0]['_release_rejection']['validation']['preflight_only']
    assert '_release_rejection' not in candidates[1]
    assert request['carry']['payload']=={'id':'held'}


def test_success_stops_before_screening_unused_candidate_windows(monkeypatch):
    from types import SimpleNamespace
    from hei_agent_bridge.arm import place
    from hei_agent_bridge.grasp import planner_backend
    request={'arguments':{'region':{'id':'r'}},'profile':{},'urdf':'unused','state':{'right':[0.]*6}}
    candidates=[{**request,'arguments':{**request['arguments'],'x_m':i,'y_m':0.}} for i in range(16)]
    planner=SimpleNamespace(backend='curobo',fk=object(),proposals=lambda goals,seed:np.zeros((len(goals),8,6)))
    monkeypatch.setattr(planner_backend,'get_planner',lambda *args:planner)
    monkeypatch.setattr(place,'region_place_candidates',lambda r:iter(candidates))
    monkeypatch.setattr(place,'placement_targets',lambda r:(None,None,None,np.eye(4),0.))
    batches=[];planned=[]
    monkeypatch.setattr(place,'batch_release_screen',lambda r,cs,*a:
                        (batches.append(len(cs)) or dict(configurations=len(cs),rejected_candidates=0)))
    monkeypatch.setattr(place,'make_place_plan',lambda r:(planned.append(1) or {}))
    monkeypatch.setattr(place,'validate_place_plan',lambda *a,**kw:{'status':'passed'})
    result=place.compute_region_place(request,{})
    assert batches==[] and len(planned)==1
    assert result['validation']['status']=='passed'


def test_cpu_refined_release_is_screened_before_path_generation(monkeypatch):
    from types import SimpleNamespace
    from hei_agent_bridge.arm import place
    from hei_agent_bridge.grasp import planner_backend
    request={'urdf':str(URDF),'state':{'right':[0.]*6},'profile':{'ik_backend':'curobo'},
             'arguments':{},'carry':{'payload':{'id':'held'}},'_collision_scene':{}}
    proposals=np.ones((8,6));refined=np.arange(6)/10
    def choose(*args):raise ValueError('GPU proposals inaccurate')
    planner=SimpleNamespace(backend='curobo',proposals=lambda *args:np.array([proposals]),choose=choose,
                            solve_proposed=lambda *args:refined)
    monkeypatch.setattr(planner_backend,'get_planner',lambda *args:planner)
    monkeypatch.setattr(place,'placement_targets',lambda *args:(*[np.eye(4)]*4,.01))
    monkeypatch.setattr(place,'base_from_frame',lambda *args:np.eye(4))
    def screen(r,p,goal,solutions,*args):
        np.testing.assert_allclose(solutions[-1],refined)
        np.testing.assert_allclose(solutions[:-1],proposals)
        raise RuntimeError('Screened before path IK')
    monkeypatch.setattr(place,'release_preflight',screen)
    monkeypatch.setattr(place,'make_plan',lambda *args:pytest.fail('Unscreened path IK'))
    with pytest.raises(RuntimeError,match='Screened before path IK'):
        place.make_place_plan(request)
