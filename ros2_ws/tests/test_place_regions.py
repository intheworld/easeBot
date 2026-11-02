"""Placement regions: conservative RGB-D geometry, skill composition and invalidation."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from test_arm_motion import make_arm_service,call,URDF
from test_arm_transport import attach,service_with_place_worker
from hei_agent_bridge.arm.regions import region_geometry,region_obstacles
from hei_agent_bridge.arm.place import region_place_candidates,make_place_plan
from hei_agent_bridge.arm.geometry import base_from_frame
from hei_agent_bridge.grasp.place_region import estimate_region,largest_rectangle


def cloud(container=True,occluded=False):
    x,y=np.meshgrid(np.arange(-.15,.151,.003),np.arange(-.12,.121,.003))
    floor=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.4)))
    if occluded:floor=floor[np.abs(floor[:,0])>.08]
    if not container:return floor
    a=np.arange(-.17,.171,.003);b=np.arange(-.14,.141,.003)
    rim=np.concatenate([np.column_stack((a,np.full(len(a),v),np.full(len(a),.47))) for v in (-.14,.14)]+
                       [np.column_stack((np.full(len(b),v),b,np.full(len(b),.47))) for v in (-.17,.17)])
    return np.concatenate([floor,rim])


def test_largest_rectangle_never_spans_unobserved_hole():
    mask=np.ones((10,12),bool);mask[:,5:7]=False
    x0,y0,x1,y1=largest_rectangle(mask)
    assert (x1-x0)*(y1-y0)==50 and mask[y0:y1,x0:x1].all()


@pytest.mark.parametrize('relation',['on','inside'])
def test_visible_region_produces_floor_and_four_conservative_walls(relation):
    region=estimate_region(cloud(relation=='inside'),np.eye(4),relation)
    region['id']='test';obstacles=region_obstacles(region,[0]*4)
    assert region['T_footprint_region'][2][3]==pytest.approx(.4)
    assert min(region['size_xy_m'])>.15
    assert len(obstacles)==(6 if relation=='inside' else 1)
    assert all(min(o['size_m'])>0 for o in obstacles if 'size_m' in o)
    if relation=='inside':assert region['wall_height_m']==pytest.approx(.07)


def test_hidden_bottom_solid_block_and_ambiguous_no_rim_are_rejected():
    with pytest.raises(ValueError,match='rim|bottom'):estimate_region(cloud(False),np.eye(4),'inside')
    points=cloud(True);points=points[points[:,2]>.45]
    with pytest.raises(ValueError):estimate_region(points,np.eye(4),'inside')


def test_support_holes_are_not_filled_with_an_invented_plane():
    region=estimate_region(cloud(False,True),np.eye(4),'on')
    assert min(region['size_xy_m'])<.08


def declared(service,backend,obj,relation='on'):
    footprint=np.linalg.inv(base_from_frame(URDF,backend.state,'base_footprint'))
    center=footprint@np.r_[obj[:3,3],1.]
    mount=np.asarray(service.carry.payload['T_tcp_object'])
    rot=footprint[:3,:3]@obj[:3,:3]
    extent=np.abs(rot[2])@np.asarray(service.carry.payload['size_m'])/2
    return dict(frame_id='base_footprint',relation=relation,geometry_confirmed=True,
        x_m=float(center[0]),y_m=float(center[1]),support_z_m=float(center[2]-extent-.006),
        size_x_m=.15,size_y_m=.15,yaw_rad=0.,wall_height_m=.03 if relation=='inside' else 0.,boundary_thickness_m=.005)


def plan_args(id):return dict(side='right',region_id=id,approach_height_m=.015,retreat_height_m=.015,release_gap_m=.08)


@pytest.mark.parametrize('alternative',[False,True])
def test_manual_region_and_old_coordinate_entry_share_place_planner(tmp_path,alternative):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path);obj=attach(service,backend)
        # Test geometry selection with real IK; collision validation is separately exercised on compute host.
        async def worker(directory):
            request=json.loads((directory/'request.json').read_text())
            if alternative:
                assert request['arguments']['region']['size_xy_m']==[.15,.15]
                assert request['arguments']['region']['selected_rectangle_option']==1
                assert request['carry']['obstacles']==region_obstacles(request['arguments']['region'],backend.state['wheels'])
            candidate=next(region_place_candidates(request))
            return dict(plan=make_place_plan(candidate),validation={'status':'passed'})
        service._worker=worker
        created=await call(skills,'create_place_region',declared(service,backend,obj),'region')
        assert created['status']=='succeeded',created
        if alternative:
            region=service.regions[created['outputs']['region_id']]['region']
            region['usable_rectangle_options']=[dict(size_xy_m=[.15,.15],
                T_footprint_region=deepcopy(region['T_footprint_region']))]
            # Too narrow even for the retained COM margin, so select an alternative.
            region['size_xy_m']=[.015,.25]
        planned=await call(skills,'plan_object_placement',plan_args(created['outputs']['region_id']),'plan')
        assert planned['status']=='succeeded' and planned['outputs']['validated'],planned
        assert not backend.calls
        entry=service.plans[planned['outputs']['plan_id']]
        assert 'region_entry' in entry and len(entry['data']['plan']['carry']['obstacles'])==1
        assert service.carry.obstacles[0]['kind']=='place_region'
        service.regions[created['outputs']['region_id']]['created']-=301
        entry['region_entry']['created']-=301
        executed=await call(skills,'execute_place_plan',dict(plan_id=planned['outputs']['plan_id']),'expired')
        assert executed['status']=='failed' and 'expired' in executed['message'] and not backend.calls
    asyncio.run(run())


def test_region_expires_and_base_motion_requires_recapture(tmp_path):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path);obj=attach(service,backend)
        created=await call(skills,'create_place_region',declared(service,backend,obj),'region')
        id=created['outputs']['region_id'];backend.state['wheels'][0]+=.1
        planned=await call(skills,'plan_object_placement',plan_args(id),'moved')
        assert planned['status']=='failed' and 'scene moved' in planned['message'] and not backend.calls
        backend.state['wheels'][0]-=.1;service.regions[id]['created']-=301
        planned=await call(skills,'plan_object_placement',plan_args(id),'old')
        assert planned['status']=='failed' and 'expired' in planned['message']
    asyncio.run(run())


def test_object_must_fit_inner_region_and_rim_increases_transfer_height(tmp_path):
    service,_,backend=make_arm_service(tmp_path);obj=attach(service,backend)
    args=declared(service,backend,obj,'inside')
    region=region_geometry(np.eye(4),*[args[k] for k in ('x_m','y_m','support_z_m','size_x_m','size_y_m','yaw_rad',
                                                       'relation','wall_height_m','boundary_thickness_m')])
    region['id']='test'
    request=dict(urdf=str(URDF),profile=service.profile,state=backend.state,carry=service.carry.snapshot(),
                 arguments={**plan_args('test'),'region':region,'path_mode':'linear'})
    candidate=next(region_place_candidates(request));plan=make_place_plan(candidate)
    goal=np.asarray(plan['T_goal']);high=max(np.asarray(p)[2,3] for p in plan['waypoint_targets'])
    assert high>=goal[2,3]+args['wall_height_m']+.015-1e-8
    region['size_xy_m']=[.015,.015]
    with pytest.raises(ValueError,match='does not fit'):next(region_place_candidates(request))


def test_new_graphs_compile_against_discovered_scalar_catalog(tmp_path):
    from test_grasp_skills import make_service
    from hei_agent_bridge.skills import SkillSet
    from hei_agent_bridge.vision import CameraSkills
    from hei_agent_bridge.cache import RobotCache
    from hei_agent.catalog import SkillCatalog
    from hei_agent.graph import Graph
    from hei_agent.planner import validate_plan
    (tmp_path/'grasp').mkdir();(tmp_path/'arm').mkdir()
    grasp,_,_,_=make_service(tmp_path/'grasp')
    arm,_,backend=make_arm_service(tmp_path/'arm');arm.region_perception=grasp
    skills=SkillSet(dict(backend='ros',named_poses={},gripper_max_gap_m=.1),backend,
                    arm=arm,grasp=grasp,cameras=CameraSkills(RobotCache()))
    catalog=SkillCatalog.from_manifest(skills.manifest())
    inputs=dict(side='right',target='blue block',destination='open box',relation='inside',approach_height_m=.04,
                retreat_height_m=.04,release_gap_m=.09,geometry_confirmed=True,frame_id='base_footprint',
                x_m=.3,y_m=-.2,support_z_m=.4,size_x_m=.2,size_y_m=.15,yaw_rad=0.,wall_height_m=.06,boundary_thickness_m=.005)
    folder=Path(__file__).resolve().parents[2]/'agent/src/hei_agent/examples'
    for name in ('right_place_region','right_place_region_plan','right_pick_place_region','right_declared_place_region_plan'):
        validate_plan(Graph.load(folder/(name+'.json')),inputs,catalog)
    with pytest.raises(Exception):validate_plan(Graph.load(folder/'right_place_region_plan.json'),{},catalog)


def test_perception_region_is_immutable_and_no_motion_and_scene_change_rejected(tmp_path):
    from test_grasp_skills import make_service
    async def run():
        (tmp_path/'grasp').mkdir();(tmp_path/'arm').mkdir()
        grasp,_,_,snapshot=make_service(tmp_path/'grasp')
        arm,_,backend=make_arm_service(tmp_path/'arm');arm.region_perception=grasp
        from hei_agent_bridge.skills import SkillSet
        skills=SkillSet(dict(backend='ros',named_poses={},gripper_max_gap_m=.1),backend,arm=arm)
        async def worker(directory):
            request=json.loads((directory/'request.json').read_text())
            assert request['operation']=='perceive_region'
            return dict(region=estimate_region(cloud(),np.eye(4),'inside'))
        grasp._worker=worker
        perceived=await call(skills,'perceive_place_region',dict(observation_id='snapshot',target='open box',relation='inside'),'perceive')
        assert perceived['status']=='succeeded',perceived
        assert not backend.calls
        entry=arm.regions[perceived['outputs']['region_id']]
        assert entry['ttl']==180 and Path(perceived['outputs']['region_ref']).is_file()
        backend.state['lift']+=.02
        with pytest.raises(RuntimeError,match='scene moved'):arm.check_region(entry,backend.state)
    asyncio.run(run())


@pytest.mark.parametrize('obstructed',[False,True])
def test_full_robot_region_floor_wall_and_retreat_collision_validation(obstructed):
    pytest.importorskip('mujoco')
    from hei_agent_bridge.arm.planner import compute
    root=Path(__file__).resolve().parents[2]
    fixture=json.loads((root/'ros2_ws/tests/fixtures/right_transport.json').read_text())
    profile=json.loads((root/'ros2_ws/src/hei_agent_bridge/config/arm_motion.json').read_text())
    args=fixture['arguments'];state=fixture['state']
    base=base_from_frame(URDF,state,args['frame_id']);footprint=np.linalg.inv(base_from_frame(URDF,state,'base_footprint'))
    point=footprint@base@np.array([args['x_m'],args['y_m'],args['support_z_m'],1.])
    region=region_geometry(np.eye(4),*point[:3],.15,.15,0.,'inside',.025,.005);region['id']='region'
    carry=deepcopy(fixture['carry']);carry['obstacles']+=region_obstacles(region,state['wheels'])
    if obstructed:
        # Solid insert across the entire usable volume: every candidate must fail.
        insert=np.asarray(region['T_footprint_region']).copy();insert[2,3]+=.06
        carry['obstacles'].append(dict(id='insert',size_m=[.15,.15,.12],T_footprint_object=insert.tolist()))
    request=dict(urdf=str(URDF),profile=profile,state=state,carry=carry,
                 arguments={**args,'region':region,'operation':'place'})
    result=compute(request)
    assert result['validation']['status']==('failed' if obstructed else 'passed'),result['validation']
    if not obstructed:
        assert len(result['plan']['transfer']['carry']['obstacles'])==5
        assert all(stage['status']=='passed' for stage in result['validation']['stages'].values())


def test_tilted_support_rejected_and_missing_side_uses_observed_extent():
    points=cloud(False);points[:,2]+=.25*points[:,0]
    with pytest.raises(ValueError,match='tilted|surface'):estimate_region(points,np.eye(4),'on')
    points=cloud();points=points[(points[:,2]<.45)|(points[:,0]<.16)]
    region=estimate_region(points,np.eye(4),'inside')
    assert region['spatial_model']['boundary']['source']=='partial_sides_with_observed_support_crop'


@pytest.mark.parametrize('ambiguous',[False,True])
def test_rgbd_worker_uses_calibrated_projection_and_rejects_distinct_destinations(tmp_path,ambiguous):
    from types import SimpleNamespace
    import cv2
    from hei_agent_bridge.grasp.worker import compute
    from test_grasp_skills import Backend
    state=Backend().state
    optical=np.diag([1.,-1.,-1.,1.]);optical[2,3]=1.
    transform=base_from_frame(URDF,state,'base_footprint')@optical
    calibration=tmp_path/'camera.npz'
    np.savez(calibration,T_result=transform,mode=['eye_to_hand'],arm=['right'],base_frame=['a_right_base_link'])
    depth=np.full((240,320),np.nan);mask=np.zeros((240,320))
    mask[45:195,60:260]=1;depth[45:195,60:260]=.53;depth[65:175,80:240]=.6
    np.save(tmp_path/'depth.npy',depth);cv2.imwrite(str(tmp_path/'rgb.jpg'),np.zeros((240,320,3),np.uint8))
    count=2 if ambiguous else 1
    masks=np.stack([mask]*count)
    if ambiguous:
        masks[0,:,160:]=0
        masks[1,:,:160]=0
    result=SimpleNamespace(boxes=SimpleNamespace(cls=np.zeros(count),conf=np.full(count,.9)),
                           masks=SimpleNamespace(data=masks),names={0:'open box'})
    class Detector:
        def set_classes(self,names):assert names==['open box']
        def predict(self,*args,**kwargs):return [result]
    request=dict(operation='perceive_region',directory=str(tmp_path),target='open box',relation='inside',
                 profile={'model_path':'unused','scene':{}},calibration=str(calibration),urdf=str(URDF),state=state,
                 observation_id='observation',info=dict(k=[500,0,160,0,500,120,0,0,1],d=[],distortion_model=''))
    if ambiguous:
        with pytest.raises(ValueError,match='Ambiguous'):compute(request,Detector())
    else:
        region=compute(request,Detector())['region']
        assert region['frame_id']=='base_footprint' and region['relation']=='inside'
        assert region['T_footprint_region'][2][3]==pytest.approx(.4,abs=1e-6)
        assert (tmp_path/'region_cloud.npy').is_file() and (tmp_path/'region_mask.png').is_file()


def test_rectangle_options_preserve_wider_supported_choices_and_exclude_holes():
    from hei_agent_bridge.grasp.place_region import rectangle_candidates
    mask=np.zeros((18,18),bool);mask[1:17,1:8]=True;mask[5:14,1:16]=True
    options=rectangle_candidates(mask)
    assert any(x1-x0>=15 and y1-y0>=9 for x0,y0,x1,y1 in options)
    assert any(x1-x0>=7 and y1-y0>=16 for x0,y0,x1,y1 in options)
    for x0,y0,x1,y1 in options:assert mask[y0:y1,x0:x1].all()


def test_payload_region_selection_uses_matching_collision_geometry(tmp_path):
    from hei_agent_bridge.arm.place import select_payload_region
    service,_,backend=make_arm_service(tmp_path);obj=attach(service,backend)
    args=declared(service,backend,obj,'inside')
    region=region_geometry(np.eye(4),*[args[k] for k in ('x_m','y_m','support_z_m','size_x_m','size_y_m','yaw_rad',
                                                       'relation','wall_height_m','boundary_thickness_m')])
    region['id']='choice';region['size_xy_m']=[.02,.25]
    option=dict(T_footprint_region=deepcopy(region['T_footprint_region']),size_xy_m=[.15,.15],
                outer_bounds_xy_m=[-.1,-.1,.1,.1])
    option['T_footprint_region'][0][3]+=.02
    region['usable_rectangle_options']=[option]
    selected=select_payload_region(region,URDF,backend.state,service.carry.payload,.003)
    assert selected['size_xy_m']==[.15,.15] and selected['selected_rectangle_option']==1
    assert region['size_xy_m']==[.02,.25]
    assert selected['T_footprint_region']==option['T_footprint_region']
    obstacles=region_obstacles(selected,[0]*4)
    assert len(obstacles)==5 and all(min(o['size_m'])>0 for o in obstacles)


def test_closing_does_not_accidentally_erode_outer_boundary_twice():
    region=estimate_region(cloud(False),np.eye(4),'on')
    # 30x24cm measured plane minus one8mm cell at each edge and quantile trimming.
    assert min(region['size_xy_m'])>=.208
    assert max(region['size_xy_m'])>=.272


def test_container_boundary_is_independent_of_bottom_holes():
    complete=estimate_region(cloud(),np.eye(4),'inside')
    holes=estimate_region(cloud(occluded=True),np.eye(4),'inside')
    assert holes['size_xy_m']==pytest.approx(complete['size_xy_m'],abs=1e-6)
    assert holes['T_footprint_region']==complete['T_footprint_region']
    c=holes['spatial_model']['coverage']
    assert c['unknown_area_m2']>complete['spatial_model']['coverage']['unknown_area_m2']
    assert sum(c[k] for k in ('unknown_area_m2','observed_support_area_m2','occupied_area_m2'))==pytest.approx(np.prod(holes['size_xy_m']))


def test_container_obstacles_do_not_shrink_boundary_and_are_kept_separate():
    x,y=np.meshgrid(np.arange(-.025,.026,.003),np.arange(-.025,.026,.003))
    obstacle=np.c_[x.ravel(),y.ravel(),np.full(x.size,.42)]
    region=estimate_region(np.concatenate([cloud(),obstacle]),np.eye(4),'inside');region['id']='observed'
    assert region['size_xy_m']==pytest.approx([.336,.276],abs=.001)
    data=region['spatial_model']['coverage']
    assert data['occupied_area_m2']>0
    boxes=region_obstacles(region,[0]*4)
    assert len(boxes)==6 and boxes[-1]['kind']=='place_region_coverage'
    assert {b['kind'] for b in boxes[-1]['boxes']}=={'unknown','occupied'}
    from hei_agent_bridge.arm.spatial import supported_centers
    candidates=list(supported_centers(region,np.array([.035,.035]),np.array([.1,.1])))
    assert candidates and all(np.max(np.abs(c))>.05 for c in candidates)


def test_exclusion_tessellation_exactly_preserves_grid_holes():
    from hei_agent_bridge.arm.spatial import exclusion_boxes
    region=estimate_region(cloud(occluded=True),np.eye(4),'inside')
    c=region['spatial_model']['coverage'];grid=np.asarray(c['cells']);origin=np.array(c['origin_xy_m']);step=c['resolution_m']
    boxes=exclusion_boxes(region)
    for y,x in np.ndindex(grid.shape):
        point=origin+(np.array([x,y])+.5)*step
        if np.any(point>=np.array(region['size_xy_m'])/2):continue
        matches=[]
        for box in boxes:
            center=np.asarray(box['T_region_box'])[:2,3];half=np.asarray(box['size_m'])[:2]/2
            if np.all(abs(point-center)<half+1e-9):matches.append(box['kind'])
        assert matches==([] if grid[y,x]==1 else ['unknown' if grid[y,x]==0 else 'occupied'])


def test_missing_side_crops_to_observed_support_instead_of_extrapolating():
    points=cloud();points=points[~((points[:,0]>.16)&(points[:,2]>.45))]
    region=estimate_region(points,np.eye(4),'inside')
    sides=region['spatial_model']['boundary']['sides']
    missing=[s for s in sides if not s['boundary_observed']]
    assert missing
    assert all(s['source']=='observed_support_extent' for s in missing)
    assert region['size_xy_m'][0]<np.ptp(points[:,0])


def test_container_sampling_rejects_unknown_without_reporting_small_size(tmp_path):
    service,_,backend=make_arm_service(tmp_path);attach(service,backend)
    region=estimate_region(cloud(),np.eye(4),'inside')
    region['spatial_model']['coverage']['cells']=np.zeros_like(region['spatial_model']['coverage']['cells']).tolist()
    request=dict(urdf=str(URDF),state=backend.state,profile=service.profile,carry=service.carry.snapshot(),arguments={'region':region})
    with pytest.raises(ValueError,match='fits container.*no support footprint'):
        list(region_place_candidates(request))


def test_coverage_collision_geometry_contains_unknown_and_observed_bodies():
    mujoco=pytest.importorskip('mujoco')
    from hei_agent_bridge.arm.carry_geometry import add_geometry
    spec=mujoco.MjSpec();spec.worldbody.add_body(name='base_footprint')
    region=estimate_region(cloud(occluded=True),np.eye(4),'inside');region['id']='unit'
    add_geometry(spec,{'carry':{'obstacles':region_obstacles(region,[0]*4)}},{})
    model=spec.compile()
    assert any('_coverage_unknown_' in model.body(i).name for i in range(model.nbody))


def test_old_bridge_cannot_validate_new_model_without_coverage_obstacles():
    from hei_agent_bridge.arm.place import compute_region_place
    region=estimate_region(cloud(occluded=True),np.eye(4),'inside');region['id']='upgrade'
    request=dict(arguments={'region':region},state={'wheels':[0]*4},
                 carry={'obstacles':region_obstacles(region,[0]*4)[:5]})
    with pytest.raises(ValueError,match='restart agent_bridge'):
        compute_region_place(request,{})


def test_model_consistency_ignores_separate_wheel_sampling_anchors(monkeypatch):
    from hei_agent_bridge.arm import place
    region=estimate_region(cloud(),np.eye(4),'inside');region['id']='anchors'
    obstacles=region_obstacles(region,[.0001]*4)
    request=dict(arguments={'region':region},state={'wheels':[0.]*4},carry={'obstacles':obstacles})
    monkeypatch.setattr(place,'region_place_candidates',lambda request:iter(()))
    with pytest.raises(ValueError,match='No reachable placement candidate'):
        place.compute_region_place(request,{})
    obstacles[0]['T_footprint_object'][0][3]+=1e-17
    with pytest.raises(ValueError,match='No reachable placement candidate'):
        place.compute_region_place(request,{})
    obstacles[0]['size_m'][0]+=.01
    with pytest.raises(ValueError,match='collision model missing or stale'):
        place.compute_region_place(request,{})


@pytest.mark.parametrize('opening_ok',[False,True])
def test_region_fast_rejection_never_skips_checks_for_passed_plan(monkeypatch,opening_ok):
    import hei_agent_bridge.arm.place as module
    calls=[]
    plan={'nominal_duration_sec':10.,'released_object_id':'object',
          **{name:{'name':name} for name in ('transfer','opening','retreat')}}
    def validate(stage,*args):
        calls.append(stage['name'])
        return {'status':'failed' if stage['name']=='opening' and not opening_ok else 'passed'}
    monkeypatch.setattr(module,'validate_arm_plan',validate)
    result=module.validate_place_plan(plan,None,{},stop_on_failure=True)
    assert calls==(['opening','transfer','retreat'] if opening_ok else ['opening'])
    assert result['status']==('passed' if opening_ok else 'failed')
    assert result['unchecked_stages']==([] if opening_ok else ['transfer','retreat'])


@pytest.mark.parametrize('validated',[False,True])
def test_expiry_does_not_hide_failed_collision_but_still_blocks_valid_plan(tmp_path,validated):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path);obj=attach(service,backend)
        service.profile['ik_backend']='scipy'
        service.binding=service._binding()
        created=await call(skills,'create_place_region',declared(service,backend,obj),'region')
        region_id=created['outputs']['region_id']
        async def worker(directory):
            request=json.loads((directory/'request.json').read_text())
            plan=make_place_plan(next(region_place_candidates(request)))
            service.regions[region_id]['created']-=400
            return dict(plan=plan,validation={'status':'passed' if validated else 'failed','stages':{
                'opening':{'status':'passed' if validated else 'failed','collision_pairs':[
                    {'bodies':['finger','tray_wall'],'constraint_kind':'observed_geometry'}]}}})
        service._worker=worker
        result=await call(skills,'plan_object_placement',plan_args(region_id),'plan')
        assert result['status']=='failed'
        assert ('expired' if validated else 'opening: observed_geometry') in result['message']
        if not validated:assert 'expired' not in result['message']
        assert not backend.calls
    asyncio.run(run())


def test_on_support_allows_four_degree_fit_and_reports_threshold():
    points=cloud(False);points[:,2]+=np.tan(np.deg2rad(4))*points[:,0]
    diagnostics={}
    region=estimate_region(points,np.eye(4),'on',diagnostics=diagnostics)
    assert region['support_point_count']>80
    assert diagnostics['support_plane']['tilt_deg']==pytest.approx(4.)
    assert diagnostics['support_plane']['max_tilt_deg']==10.
