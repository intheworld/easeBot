"""General TCP skills: geometry, data contracts and mock ROS execution ownership."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import sys
import time

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.arm.geometry import FRAMES, base_from_frame, pose_fields, quaternion_matrix
from hei_agent_bridge.arm.planner import make_plan, trajectory_samples
from hei_agent_bridge.arm.service import ArmMotionSkills
from hei_agent_bridge.grasp.kinematics import RightArmFK
from hei_agent_bridge.skills import SkillSet
from test_grasp_skills import Backend

URDF = ROOT/'ros2_ws/src/hei_description/urdf/hei_body.urdf'


class ArmBackend(Backend):
    arm_scene_state = Backend.grasp_state
    arm_pose_state = Backend.grasp_state
    async def arm_trajectory(self, invocation, start, waypoints, speed):
        self.calls.append(('arm',invocation,deepcopy(waypoints)))
        await asyncio.sleep(self.delay)
        self.state['right'] = list(waypoints[-1])


def make_arm_service(tmp_path):
    profile = json.loads((ROOT/'ros2_ws/src/hei_agent_bridge/config/arm_motion.json').read_text())
    profile['worker_python'] = sys.executable
    backend = ArmBackend()
    service = ArmMotionSkills(profile,backend,directory=tmp_path/'plans',urdf=URDF)
    async def compute(directory):
        # Orchestration tests stub only the collision screen, not IK or transforms.
        request = json.loads((directory/'request.json').read_text())
        return {'plan':make_plan(request),'validation':{'status':'passed'}}
    service._worker = compute
    skills = SkillSet({'backend':'ros','named_poses':{},'gripper_max_gap_m':.1},backend,arm=service)
    return service,skills,backend


def call(skills,name,args,id):
    return skills.invoke(name,args,invocation=id,revision=skills.revision)


def target(service,backend,frame='a_right_base_link',path='joint',move=True):
    q = np.array(backend.state['right'])
    if move:q += [.01,-.01,.01,0.,.01,0.]
    pose = np.linalg.inv(base_from_frame(URDF,backend.state,frame)) @ service.fk(q)
    fields = pose_fields(pose,frame)
    return {k:v for k,v in fields.items() if k != 'tool_frame'} | {'path_mode':path}


@pytest.mark.parametrize('height',[0.,-.149365,-.3])
@pytest.mark.parametrize('frame',FRAMES)
def test_pose_frame_roundtrip_preserves_tool_at_all_lift_heights(height,frame):
    state = Backend().state
    state['lift'] = height
    fk = RightArmFK(URDF)
    base_pose = fk(state['right'])
    transform = base_from_frame(URDF,state,frame)
    fields = pose_fields(np.linalg.inv(transform)@base_pose,frame)
    rebuilt = np.eye(4)
    rebuilt[:3,3] = [fields[k] for k in ('x_m','y_m','z_m')]
    rebuilt[:3,:3] = quaternion_matrix([fields[k] for k in ('qx','qy','qz','qw')])
    np.testing.assert_allclose(transform@rebuilt,base_pose,atol=1e-8)


@pytest.mark.parametrize('q',[[0,0,0,0],[0,0,0,2],[float('nan'),0,0,1]])
def test_invalid_quaternions_are_rejected(q):
    with pytest.raises(ValueError,match='unit xyzw'):
        quaternion_matrix(q)


@pytest.mark.parametrize('frame',FRAMES)
def test_read_tcp_pose_does_not_dispatch_motion(tmp_path,frame):
    async def run():
        service,skills,backend = make_arm_service(tmp_path)
        result = await call(skills,'get_arm_pose',{'side':'right','frame_id':frame},'pose')
        assert result['status']=='succeeded',result
        assert result['outputs']['frame_id']==frame and result['outputs']['tool_frame']=='a_right_tcp'
        assert not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('mode',['joint','linear'])
@pytest.mark.parametrize('frame',FRAMES)
def test_pose_plan_executes_arm_only_and_is_single_use(tmp_path,mode,frame):
    async def run():
        service,skills,backend = make_arm_service(tmp_path)
        before = deepcopy(backend.state)
        args = target(service,backend,frame,mode)
        result = await call(skills,'plan_arm_pose',args,'plan')
        assert result['status']=='succeeded',result
        assert not backend.calls
        id = result['outputs']['plan_id']
        assert result['outputs']['validated']
        result = await call(skills,'execute_arm_plan',{'plan_id':id},'execute')
        assert result['status']=='succeeded',result
        assert result['outputs']['reached_goal'] and result['outputs']['position_error_m'] < .005
        assert len(backend.calls)==1 and backend.calls[0][0]=='arm'
        assert backend.state['right_gripper']==before['right_gripper']
        assert backend.state['left']==before['left']
        result = await call(skills,'execute_arm_plan',{'plan_id':id},'replay')
        assert result['status']=='failed' and len(backend.calls)==1
    asyncio.run(run())


def test_position_target_preserves_current_orientation(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        pose=service.fk(backend.state['right'])
        # Current position is also a useful no-displacement smoke test.
        args={'side':'right','frame_id':'a_right_base_link','path_mode':'linear',
              **dict(zip(('x_m','y_m','z_m'),pose[:3,3].tolist()))}
        result=await call(skills,'plan_arm_position',args,'position')
        assert result['status']=='succeeded',result
        plan=service.plans[result['outputs']['plan_id']]['data']['plan']
        np.testing.assert_allclose(np.array(plan['T_goal'])[:3,:3],pose[:3,:3])
        samples=list(trajectory_samples(plan['start_joint_rad'],plan['waypoints_rad'],.1))
        assert samples[-1][0]==pytest.approx(plan['nominal_duration_sec'])
    asyncio.run(run())


@pytest.mark.parametrize('issue',['expired','collision','start','lift','gripper','wrong_plan','model'])
def test_invalid_plans_never_dispatch(tmp_path,issue):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        result=await call(skills,'plan_arm_pose',target(service,backend),'plan')
        assert result['status']=='succeeded',result
        id=result['outputs']['plan_id'];entry=service.plans[id]
        if issue=='expired':entry['created']-=61
        if issue=='collision':entry['data']['validation']['status']='failed'
        if issue=='start':backend.state['right'][0]+=.1
        if issue=='lift':backend.state['lift']+=.02
        if issue=='gripper':backend.state['right_gripper']+=.1
        if issue=='wrong_plan':id='0'*32
        if issue=='model':service.binding='changed'
        result=await call(skills,'execute_arm_plan',{'plan_id':id},'bad')
        assert result['status']=='failed',result
        assert not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('cause',['cancel','lift','right_gripper','left','wheels'])
def test_cancel_and_scene_guard_cancel_only_owned_goal(tmp_path,cause):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        result=await call(skills,'plan_arm_pose',target(service,backend),'plan')
        assert result['status']=='succeeded',result
        id=result['outputs']['plan_id'];backend.delay=2
        task=asyncio.create_task(call(skills,'execute_arm_plan',{'plan_id':id},'motion'))
        while not backend.calls:await asyncio.sleep(.001)
        if cause=='cancel':await skills.cancel('motion')
        elif cause in ('left','wheels'):backend.state[cause][0]+=.1
        else:backend.state[cause]+=.1
        result=await task
        assert result['status']==('cancelled' if cause=='cancel' else 'failed'),result
        assert result['cleanup_complete'] and backend.cancelled==['motion']
        assert service.plans[id]['consumed']
    asyncio.run(run())


def test_inaccurate_measured_endpoint_is_not_reported_as_reached(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        result=await call(skills,'plan_arm_pose',target(service,backend),'plan')
        id=result['outputs']['plan_id']
        async def inaccurate(*args):
            backend.calls.append(('arm',))
            backend.state['right'][0]+=.2
        backend.arm_trajectory=inaccurate
        result=await call(skills,'execute_arm_plan',{'plan_id':id},'motion')
        assert result['status']=='failed' and 'Measured TCP target not reached' in result['message']
    asyncio.run(run())


def test_arm_catalog_and_packaged_graphs_match(tmp_path):
    from hei_agent.catalog import SkillCatalog
    from hei_agent.graph import Graph
    from hei_agent.planner import validate_plan
    service,skills,_=make_arm_service(tmp_path)
    catalog=SkillCatalog.from_manifest(skills.manifest())
    assert {'get_arm_pose','plan_arm_pose','plan_arm_position','execute_arm_plan'} <= {s['name'] for s in catalog.describe()}
    for name in ('arm_pose.json','arm_position_plan.json','arm_position_execute.json'):
        validate_plan(Graph.load(ROOT/'agent/src/hei_agent/examples'/name),{},catalog)


def test_unknown_frames_sides_quaternion_and_unreachable_goal_never_dispatch(tmp_path):
    async def run():
        service,skills,backend=make_arm_service(tmp_path)
        for index,change in enumerate(({'side':'left'},{'frame_id':'world'},{'qx':0.,'qy':0.,'qz':0.,'qw':0.},{'x_m':10.})):
            args=target(service,backend)|change
            try:
                result=await call(skills,'plan_arm_pose',args,'bad'+str(index))
                assert result['status']=='failed',result
            except ValueError:
                pass
        assert not backend.calls
    asyncio.run(run())



def test_arm_profile_does_not_need_yolo_or_hand_eye(tmp_path,monkeypatch):
    import hei_agent_bridge.arm.service as module
    profile=json.loads((ROOT/'ros2_ws/src/hei_agent_bridge/config/arm_motion.json').read_text())
    profile['worker_python']=sys.executable
    (tmp_path/'arm_motion.json').write_text(json.dumps(profile))
    monkeypatch.setattr(module,'data_directory',lambda:tmp_path)
    prepared=module.prepare_config({})
    assert 'model_path' not in prepared and 'calibration' not in prepared
    ArmMotionSkills(prepared,ArmBackend(),directory=tmp_path/'plans',urdf=URDF)
