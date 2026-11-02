"""Transport state machine and contracts. No ROS or physical devices."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from test_arm_motion import make_arm_service,call,URDF
from hei_agent_bridge.arm.payload import CarryState
from hei_agent_bridge.arm.geometry import pose_fields,base_from_frame
from hei_agent_bridge.arm.planner import make_plan
from hei_agent_bridge.arm.place import make_place_plan


def attach(service,backend):
    tcp=service.fk(backend.state['right']);obj=tcp.copy();obj[:3,3]+=tcp[:3,0]*.03
    obb=dict(center_m=obj[:3,3].tolist(),rotation=obj[:3,:3].tolist(),size_m=[.02,.02,.02])
    service.carry.attach(obb,tcp,backend.state['right_gripper'])
    return obj


def placement_args(service,backend,obj):
    extent=float(np.abs(obj[2,:3])@np.array([.02]*3)/2)
    return dict(side='right',frame_id='a_right_base_link',x_m=float(obj[0,3]+.003),
                y_m=float(obj[1,3]),support_z_m=float(obj[2,3]-extent-service.profile['release_clearance_m']),
                approach_height_m=.015,retreat_height_m=.015,release_gap_m=.08)


def service_with_place_worker(tmp_path):
    service,skills,backend=make_arm_service(tmp_path)
    async def worker(directory):
        r=json.loads((directory/'request.json').read_text())
        plan=make_place_plan(r) if r['arguments'].get('operation')=='place' else make_plan(r)
        return dict(plan=plan,validation={'status':'passed'})
    service._worker=worker
    async def gripper(invocation,side,gap):
        backend.calls.append(('release',invocation,gap))
        backend.state['right_gripper']=-gap/abs(service.profile['gap_per_motor_rad_m'])
        return gap
    backend.gripper=gripper
    return service,skills,backend


@pytest.mark.parametrize('frame',['a_right_base_link','base_footprint','a_right_tcp'])
def test_relative_translation_preserves_orientation_and_uses_declared_axes(tmp_path,frame):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        initial=service.fk(backend.state['right'])
        result=await call(skills,'plan_arm_offset',dict(side='right',frame_id=frame,dx_m=.001,dy_m=0.,dz_m=0.),'relative')
        assert result['status']=='succeeded',result
        plan=service.plans[result['outputs']['plan_id']]['data']['plan']
        actual=np.asarray(plan['T_goal'])
        axes=initial[:3,:3] if frame=='a_right_tcp' else base_from_frame(URDF,backend.state,frame)[:3,:3]
        np.testing.assert_allclose(actual[:3,3]-initial[:3,3],axes@np.array([.001,0,0]),atol=1e-9)
        np.testing.assert_allclose(actual[:3,:3],initial[:3,:3],atol=1e-9)
        assert not backend.calls
    asyncio.run(run())


def test_multisegment_path_is_frozen_and_planned_as_one_trajectory(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        result=await call(skills,'create_arm_path',dict(side='right',frame_id='a_right_base_link',path_mode='linear'),'path')
        id=result['outputs']['path_id'];pose=service.fk(backend.state['right'])
        for i in range(2):
            target=pose.copy();target[2,3]+=.001*(i+1)
            args={k:v for k,v in pose_fields(target,'a_right_base_link').items() if k not in ('side','frame_id','tool_frame')}
            result=await call(skills,'append_arm_waypoint',dict(path_id=id,**args),'append'+str(i))
            assert result['outputs']['waypoint_count']==i+1
        result=await call(skills,'plan_arm_path',{'path_id':id},'plan')
        assert result['status']=='succeeded',result
        plan=service.plans[result['outputs']['plan_id']]['data']['plan']
        assert len(plan['waypoint_targets'])==2 and not backend.calls
        result=await call(skills,'plan_arm_path',{'path_id':id},'again')
        assert result['status']=='failed' and 'frozen' in result['message']
    asyncio.run(run())


def test_empty_expired_and_overfull_paths_are_rejected(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        created=await call(skills,'create_arm_path',dict(side='right',frame_id='base_link',path_mode='joint'),'create')
        id=created['outputs']['path_id']
        result=await call(skills,'plan_arm_path',dict(path_id=id),'empty')
        assert result['status']=='failed' and 'empty' in result['message']
        service.paths[id]['created']-=301
        result=await call(skills,'plan_arm_path',dict(path_id=id),'expired')
        assert result['status']=='failed' and 'expired' in result['message']
        assert not backend.calls
    asyncio.run(run())


def test_payload_geometry_changes_invalidate_a_previously_valid_plan(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        result=await call(skills,'plan_arm_home',{'side':'right'},'plan')
        attach(service,backend)
        result=await call(skills,'execute_arm_plan',{'plan_id':result['outputs']['plan_id']},'execute')
        assert result['status']=='failed' and 'scene changed' in result['message'] and not backend.calls
    asyncio.run(run())


def test_carry_state_is_session_memory_and_ignores_old_journal(tmp_path):
    old=tmp_path/'carry.json';old.write_text('old persistent state must not be read')
    service,_,backend=make_arm_service(tmp_path)
    attach(service,backend)
    fresh=CarryState()
    assert fresh.status=='unknown' and fresh.payload is None and fresh.obstacles==[]
    assert old.read_text()=='old persistent state must not be read'
    assert not hasattr(service.carry,'path')


def test_live_fully_open_feedback_clears_payload_without_manual_reset(tmp_path):
    service,_,backend=make_arm_service(tmp_path)
    attach(service,backend)
    service.carry.configure_gripper(-4.,-.0222222222222)
    backend.state['right_gripper']=-4.
    assert service.carry.check(backend.state)['status']=='empty'
    assert service.carry.payload is None
    assert service.carry.observation['jaw_state']=='open'


def test_small_gripper_changes_are_observations_not_sticky_failures(tmp_path):
    service,_,backend=make_arm_service(tmp_path)
    attach(service,backend)
    backend.state['right_gripper']-=.2
    assert service.carry.check(backend.state)['status']=='attached'


def test_place_executes_transfer_release_and_revalidated_retreat(tmp_path):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path)
        obj=attach(service,backend)
        planned=await call(skills,'plan_place_object',placement_args(service,backend,obj),'plan')
        assert planned['status']=='succeeded',planned
        assert not backend.calls
        result=await call(skills,'execute_place_plan',{'plan_id':planned['outputs']['plan_id']},'execute')
        assert result['status']=='succeeded',result
        assert result['outputs']['placed'] and result['outputs']['retracted']
        assert not result['outputs']['placement_verified']
        assert [c[0] for c in backend.calls]==['arm','release','arm']
        assert service.carry.status=='empty' and len(service.carry.obstacles)==1
    asyncio.run(run())


@pytest.mark.parametrize('failure',['collision','transfer','release','retreat'])
def test_place_failures_do_not_release_early_or_retry(tmp_path,failure):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path)
        obj=attach(service,backend)
        planned=await call(skills,'plan_place_object',placement_args(service,backend,obj),'plan')
        assert planned['status']=='succeeded',planned
        id=planned['outputs']['plan_id'];normal=backend.arm_trajectory
        if failure=='collision':service.plans[id]['data']['validation']['status']='failed'
        if failure=='transfer':
            async def fail(*args):raise RuntimeError('transfer failed')
            backend.arm_trajectory=fail
        if failure=='release':
            async def fail(*args):raise RuntimeError('opening failed')
            backend.gripper=fail
        if failure=='retreat':
            async def fail_worker(directory):return dict(validation={'status':'failed'})
            service._worker=fail_worker
        result=await call(skills,'execute_place_plan',dict(plan_id=id),'execute')
        assert result['status']=='failed',result
        releases=[c for c in backend.calls if c[0]=='release']
        if failure in ('collision','transfer'):assert not releases and service.carry.status=='attached'
        if failure=='release':assert service.carry.status=='unknown'
        if failure=='retreat':assert len(releases)==1 and service.carry.status=='empty' and len(service.carry.obstacles)==1
    asyncio.run(run())


def test_place_rejects_too_small_release_gap_without_dispatch(tmp_path):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path)
        obj=attach(service,backend)
        args=placement_args(service,backend,obj);args['release_gap_m']=.005
        result=await call(skills,'plan_place_object',args,'plan')
        assert result['status']=='failed' and 'gap' in result['message'] and not backend.calls
    asyncio.run(run())


def test_transport_graphs_compile_against_the_actual_catalog(tmp_path):
    from hei_agent.catalog import SkillCatalog
    from hei_agent.graph import Graph
    from hei_agent.planner import validate_plan
    from hei_agent_bridge.skills import SkillSet
    from test_grasp_skills import make_service
    (tmp_path/'arm').mkdir();(tmp_path/'grasp').mkdir()
    service,_,backend=service_with_place_worker(tmp_path/'arm')
    grasp,_,_,_=make_service(tmp_path/'grasp')
    from hei_agent_bridge.vision import CameraSkills
    from hei_agent_bridge.cache import RobotCache
    cameras=CameraSkills(RobotCache(),simulated=False)
    skills=SkillSet({'backend':'ros','named_poses':{},'gripper_max_gap_m':.1},backend,arm=service,grasp=grasp,cameras=cameras)
    catalog=SkillCatalog.from_manifest(skills.manifest())
    inputs=dict(side='right',target='blue block',frame_id='a_right_base_link',dx_m=0.,dy_m=0.,dz_m=.05,
                x_m=.4,y_m=.1,support_z_m=-.1,approach_height_m=.05,retreat_height_m=.05,release_gap_m=.09)
    for i in (1,2):inputs.update({f'p{i}_{k}':v for k,v in dict(x_m=.4,y_m=.1,z_m=.2,qx=0.,qy=0.,qz=0.,qw=1.).items()})
    root=Path(__file__).resolve().parents[2]/'agent/src/hei_agent/examples'
    for name in ('right_arm_offset','right_arm_offset_plan','right_arm_path','right_arm_path_plan','right_place_object','right_place_object_plan','right_pick_place','right_carry_state'):
        validate_plan(Graph.load(root/(name+'.json')),inputs,catalog)
    with pytest.raises(Exception):validate_plan(Graph.load(root/'right_place_object.json'),{},catalog)


@pytest.mark.parametrize('support',['table','region_floor'])
def test_transport_geometry_contacts_and_support_separation(support):
    mujoco=pytest.importorskip('mujoco')
    from hei_agent_bridge.arm.carry_geometry import add_geometry
    from hei_agent_bridge.grasp.validation import CollisionScreen
    spec=mujoco.MjSpec()
    base=spec.worldbody.add_body(name='base_footprint')
    surface=base if support=='table' else base.add_body(name='placed_test_floor')
    surface.add_geom(name='validation_table' if support=='table' else 'region_floor',type=mujoco.mjtGeom.mjGEOM_BOX,pos=[0,0,0],size=[.5,.5,.05])
    tcp=spec.worldbody.add_body(name='a_right_tcp',pos=[0,0,.072]);tcp.add_freejoint()
    tcp.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE,size=[.001],mass=.01,contype=0,conaffinity=0)
    carry=dict(status='attached',payload=dict(id='test',size_m=[.04]*3,T_tcp_object=np.eye(4).tolist()),obstacles=[])
    add_geometry(spec,dict(carry=carry),{})
    model=spec.compile();data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    screen=CollisionScreen(model,data,dict(clearance_m=0))
    screen.check('start',0)
    assert not screen.collisions and screen.support_contact is not None
    data.qpos[2]=.1;mujoco.mj_forward(model,data)
    screen.check('lift',1)
    assert screen.support_separated
    data.qpos[2]=.072;mujoco.mj_forward(model,data)
    screen.check('return',2)
    assert screen.collisions  # Initial-contact exception cannot be reused.


def test_carried_box_collides_with_arm_and_placed_box_collides_after_release():
    mujoco=pytest.importorskip('mujoco')
    from hei_agent_bridge.arm.carry_geometry import add_geometry
    from hei_agent_bridge.grasp.validation import CollisionScreen
    for attached in (True,False):
        spec=mujoco.MjSpec();base=spec.worldbody.add_body(name='base_footprint')
        tcp=spec.worldbody.add_body(name='a_right_tcp',pos=[0,0,.2]);tcp.add_freejoint()
        tcp.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE,size=[.001],mass=.01,contype=0,conaffinity=0)
        link=spec.worldbody.add_body(name='a_right_link3',pos=[0,0,.2]);link.add_freejoint()
        link.add_geom(name='arm_box',type=mujoco.mjtGeom.mjGEOM_BOX,size=[.02]*3)
        pose=np.eye(4);pose[2,3]=.2
        carry=dict(status='attached' if attached else 'empty',obstacles=[],payload=None)
        if attached:carry['payload']=dict(id='test',size_m=[.04]*3,T_tcp_object=np.eye(4).tolist())
        else:carry['obstacles']=[dict(id='test',size_m=[.04]*3,T_footprint_object=pose.tolist())]
        add_geometry(spec,dict(carry=carry),{})
        model=spec.compile();data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        screen=CollisionScreen(model,data,dict(clearance_m=0));screen.check('test',0)
        assert screen.collisions


@pytest.mark.parametrize('stage',['transfer','release','retreat'])
def test_placement_cancellation_preserves_stage_and_object_state(tmp_path,stage):
    async def run():
        service,skills,backend=service_with_place_worker(tmp_path)
        obj=attach(service,backend)
        planned=await call(skills,'plan_place_object',placement_args(service,backend,obj),'plan')
        assert planned['status']=='succeeded',planned
        entered=asyncio.Event();normal_move=backend.arm_trajectory
        calls=0
        async def moving(*args):
            nonlocal calls
            calls+=1
            if stage=='transfer' or stage=='retreat' and calls==2:
                entered.set();await asyncio.sleep(20)
            await normal_move(*args)
        backend.arm_trajectory=moving
        if stage=='release':
            async def opening(*args):entered.set();await asyncio.sleep(20)
            backend.gripper=opening
        task=asyncio.create_task(call(skills,'execute_place_plan',dict(plan_id=planned['outputs']['plan_id']),'execute'))
        await asyncio.wait_for(entered.wait(),3)
        await skills.cancel('execute');result=await task
        assert result['status']=='cancelled' and result['cleanup_complete']
        assert service.carry.status==dict(transfer='attached',release='unknown',retreat='empty')[stage]
        journal=json.loads((service.directory/planned['outputs']['plan_id']/'placement_state.json').read_text())
        assert journal['failed_stage']==stage
    asyncio.run(run())


def test_explicit_gripper_open_is_allowed_and_live_feedback_clears_model(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path);attach(service,backend)
        service.carry.configure_gripper(-4.,-.0222222222222)
        async def gripper(invocation,side,gap):
            backend.state['right_gripper']=-4.05
            return gap
        backend.gripper=gripper;backend.arm_scene_state=lambda:deepcopy(backend.state)
        result=await call(skills,'set_gripper_gap',dict(side='right',gap_m=.09),'open')
        assert result['status']=='succeeded',result
        assert service.carry.status=='empty' and service.carry.payload is None
    asyncio.run(run())


def test_manual_payload_geometry_update_is_bound_into_future_plans(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path);attach(service,backend)
        token=service.carry.token
        result=await call(skills,'set_carried_object_geometry',dict(side='right',geometry_confirmed=True,observed_attached=True,
            x_m=.03,y_m=0.,z_m=0.,qx=0.,qy=0.,qz=0.,qw=1.,size_x_m=.03,size_y_m=.02,size_z_m=.04),'shape')
        assert result['status']=='succeeded',result
        assert service.carry.payload['size_m']==[.03,.02,.04]
        assert service.carry.token!=token and not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('blocked',[False,True])
def test_full_robot_place_geometry_previews_all_phases(blocked):
    pytest.importorskip('mujoco')
    from hei_agent_bridge.arm.planner import compute
    root=Path(__file__).resolve().parents[2]
    fixture=json.loads((root/'ros2_ws/tests/fixtures/right_transport.json').read_text())
    profile=json.loads((root/'ros2_ws/src/hei_agent_bridge/config/arm_motion.json').read_text())
    request={k:fixture[k] for k in ('state','carry','arguments')}
    request.update(profile=profile,urdf=str(URDF))
    if blocked:
        preview=make_place_plan(request)
        obstacle=deepcopy(preview['opening']['carry']['obstacles'][-1]);obstacle['id']='blocker'
        request['carry']['obstacles'].append(obstacle)
    result=compute(request)
    stages=result['validation']['stages']
    if blocked:
        assert result['validation']['status']=='failed'
        assert any('placed_blocker' in pair['bodies'] for pair in stages['transfer']['collision_pairs'])
    else:
        assert result['validation']['status']=='passed',result['validation']
        assert all(stage['status']=='passed' for stage in stages.values())
        assert all(stage['samples']>0 for stage in stages.values())


def test_base_movement_invalidates_remembered_world_geometry(tmp_path):
    service,_,backend=make_arm_service(tmp_path);attach(service,backend)
    service.carry.released(np.eye(4),wheels=backend.state['wheels'])
    backend.state['wheels'][0]+=.1
    with pytest.raises(RuntimeError,match='Base moved'):service.carry.check(backend.state)


def test_regrasp_keeps_static_object_until_closure_commits(tmp_path):
    service,_,backend=make_arm_service(tmp_path)
    tcp=service.fk(backend.state['right'])
    obb=dict(center_m=tcp[:3,3].tolist(),rotation=tcp[:3,:3].tolist(),size_m=[.02]*3,known_obstacle_id='previous')
    service.carry.obstacles=[dict(id='previous',size_m=[.02]*3,T_footprint_object=np.eye(4).tolist())]
    service.carry.attach(obb,tcp,backend.state['right_gripper'],pending=True)
    assert service.carry.status=='unknown' and len(service.carry.obstacles)==1
    assert service.carry.payload['source_obstacle_id']=='previous'
    service.carry.attach(obb,tcp,backend.state['right_gripper'])
    assert service.carry.status=='attached' and not service.carry.obstacles


def test_placed_box_encloses_release_and_settled_positions(tmp_path):
    service,_,backend=make_arm_service(tmp_path);attach(service,backend)
    payload=deepcopy(service.carry.payload);mount=np.asarray(payload['T_tcp_object'])
    release_center=mount[:3,3].copy();drop=.011
    obstacle=service.carry.released(np.eye(4),drop)
    pose=np.asarray(obstacle['T_footprint_object']);half=np.asarray(obstacle['size_m'])/2
    for center in (release_center,release_center-np.array([0.,0.,drop])):
        local=pose[:3,:3].T@(center-pose[:3,3])
        assert np.all(np.abs(local)<=half+1e-9)
