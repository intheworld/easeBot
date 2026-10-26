"""Right grasp geometry and orchestration; no physical devices."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.grasp.planner import RightArmPlanner
from hei_agent_bridge.grasp.service import GraspSkills
from hei_agent_bridge.skills import SkillSet


def test_real_model_fk_ik_and_camera_transform():
    planner = RightArmPlanner(ROOT / 'ros2_ws/src/hei_description/urdf/hei_body.urdf')
    q = np.array([.4, -.5, -.7, .2, .3, .8])
    goal = planner.fk(q + [.01, -.01, .01, 0, .01, 0])
    path = planner.cartesian_path(q, goal)
    np.testing.assert_allclose(planner.fk(path[-1]), goal, atol=1e-4)
    with np.load(ROOT / 'ros2_ws/src/hei_agent_bridge/config/right_eye_to_hand.npz', allow_pickle=False) as data:
        transform = data['T_result']
        assert str(data['mode'][0]) == 'eye_to_hand'
    from hei_agent_bridge.grasp.worker import camera_transform
    np.testing.assert_allclose(camera_transform(ROOT / 'ros2_ws/src/hei_agent_bridge/config/right_eye_to_hand.npz'),transform)
    with pytest.raises(ValueError, match='IK failed'):
        planner.solve(np.diag([1., 1., 1., 1.]) + np.array([[0,0,0,10],[0,0,0,0],[0,0,0,0],[0,0,0,0]]), q)


class Backend:
    def __init__(self):
        self.state = {'right': [.4, -.5, -.7, .2, .3, .8], 'left': [0.]*6, 'lift': -.149365,
                      'left_gripper': 0., 'right_gripper': -.1, 'wheels': [0.]*4}
        self.calls, self.cancelled = [], []
        self.delay = 0
    def grasp_state(self, **kwargs):
        return deepcopy(self.state)
    def ready(self):
        return True
    async def grasp_check(self, profile):
        pass
    async def grasp_gripper(self, invocation, gap, *, close):
        self.calls.append(('close' if close else 'open', invocation, gap))
        await asyncio.sleep(self.delay)
        return close
    async def grasp_trajectory(self, invocation, start, path, speed):
        self.calls.append(('trajectory', invocation, deepcopy(path)))
        await asyncio.sleep(self.delay)
    async def cancel(self, invocation):
        self.cancelled.append(invocation)


def make_service(tmp_path):
    backend = Backend()
    snapshot = {'created': time.monotonic(), 'simulated': False, 'camera': 'front',
                'depth': np.ones((3,3), dtype=np.float32),
                'info': {'frame_id': 'front_camera_optical_frame'}, 'assets': {'rgb': ('image/jpeg', b'rgb')}}
    cameras = SimpleNamespace(_get=lambda _: snapshot)
    profile = json.loads((ROOT / 'ros2_ws/src/hei_agent_bridge/config/right_grasp.json').read_text())
    model = tmp_path / 'weights.pt'; model.write_bytes(b'test')
    profile.update(model_path=str(model), worker_python=sys.executable)
    service = GraspSkills(profile, cameras, backend, directory=tmp_path/'plans',
                          urdf=ROOT / 'ros2_ws/src/hei_description/urdf/hei_body.urdf')
    async def compute(directory):
        request = json.loads((directory/'request.json').read_text())
        if request['operation']=='perceive':
            return {'object':{'schema':'hei.obb/v1','frame_id':'a_right_base_link',
                'target':request['target'],'confidence':.9,'point_count':100,
                'geometry_scope':'visible_surface_only','center_m':[.4,.1,.2],
                'size_m':[.06,.03,.04],'rotation':np.eye(3).tolist()}}
        plan = {'schema_version':3,'strategy':'top_down_direct_ik',
                'T_grasp': np.eye(4).tolist(), 'pregrasp_joint_rad': [.1]*6,
                'approach_joint_rad': [[.2]*6]}
        return {'plan': plan, 'validation': {'status': 'passed'}}
    service._worker = compute
    config = {'backend': 'ros', 'named_poses': {}, 'gripper_max_gap_m': .1}
    skills = SkillSet(config, backend, grasp=service)
    return service, skills, backend, snapshot


def invoke(skills, name, args, id):
    return skills.invoke(name, args, invocation=id, revision=skills.revision)


async def make_obb_plan(skills):
    observed=await invoke(skills,'perceive_grasp_target',{'observation_id':'image','target':'cube'},'detect')
    assert observed['status']=='succeeded',observed
    return await invoke(skills,'plan_right_grasp_from_object',{'object_id':observed['outputs']['object_id']},'plan')


def test_grasp_plan_execute_single_use_and_current_limited_close(tmp_path):
    async def run():
        service, skills, backend, _ = make_service(tmp_path)
        result = await make_obb_plan(skills)
        assert result['status'] == 'succeeded', result
        assert not backend.calls
        plan_id = result['outputs']['plan_id']
        result = await invoke(skills, 'execute_right_grasp', {'plan_id':plan_id}, 'execute')
        assert result['status'] == 'succeeded', result
        assert result['outputs'] == {'plan_id':plan_id, 'completed':True, 'gripper_stalled':True, 'grasp_verified':False}
        assert [v[0] for v in backend.calls] == ['open','trajectory','trajectory','close']
        result = await invoke(skills, 'execute_right_grasp', {'plan_id':plan_id}, 'replay')
        assert result['status'] == 'failed' and 'consumed' in result['message']
        assert len(backend.calls) == 4
    asyncio.run(run())


@pytest.mark.parametrize('problem', ['expired','collision','pose','scene','model','format'])
def test_no_dispatch_on_invalid_plan(tmp_path, problem):
    async def run():
        service, skills, backend, _ = make_service(tmp_path)
        result = await make_obb_plan(skills)
        id = result['outputs']['plan_id']; entry = service.plans[id]
        if problem == 'expired': entry['created'] -= 61
        if problem == 'collision': entry['data']['validation']['status'] = 'failed'
        if problem == 'format': entry['data']['plan']['schema_version'] = 2
        if problem == 'pose': backend.state['right'][0] += .1
        if problem == 'scene': backend.state['left'][0] += .1
        if problem == 'model': Path(service.profile['model_path']).write_bytes(b'changed')
        result = await invoke(skills, 'execute_right_grasp', {'plan_id':id}, 'bad')
        assert result['status'] == 'failed', result
        assert not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('problem', ['simulated', 'stale', 'wrong_frame'])
def test_real_aligned_recent_snapshot_required(tmp_path, problem):
    async def run():
        service, skills, backend, snapshot = make_service(tmp_path)
        if problem == 'simulated': snapshot['simulated'] = True
        if problem == 'stale': snapshot['created'] -= 4
        if problem == 'wrong_frame': snapshot['info']['frame_id'] = 'wrong'
        result = await invoke(skills, 'perceive_grasp_target', {'observation_id':'image','target':'cube'}, 'bad')
        assert result['status'] == 'failed' and not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('cause', ['cancel', 'base', 'lift'])
def test_cancel_and_scene_motion_stop_subsequent_phases(tmp_path, cause):
    async def run():
        service, skills, backend, _ = make_service(tmp_path)
        result = await make_obb_plan(skills)
        id = result['outputs']['plan_id']
        backend.delay = 2
        task = asyncio.create_task(invoke(skills, 'execute_right_grasp', {'plan_id':id}, 'execute'))
        while not backend.calls:
            await asyncio.sleep(.001)
        if cause == 'cancel':
            await skills.cancel('execute')
        elif cause == 'lift':
            backend.state['lift'] += .02
        else:
            backend.state['wheels'][0] += .1
        result = await task
        assert result['status'] == ('cancelled' if cause == 'cancel' else 'failed'), result
        assert result['cleanup_complete']
        assert backend.cancelled == ['execute']
        assert len(backend.calls) == 1
        assert service.plans[id]['consumed']
    asyncio.run(run())


def test_catalog_binding_changes_for_calibration_or_model(tmp_path):
    service, skills, _, _ = make_service(tmp_path)
    before = skills.revision
    Path(service.profile['model_path']).write_bytes(b'changed')
    service.binding = service._binding()
    after = SkillSet(skills.config, skills.backend, grasp=service)
    assert after.revision != before


def test_compute_process_cancel_is_joined_before_cleanup(tmp_path):
    import os
    async def run():
        service, _, _, _ = make_service(tmp_path)
        pid_path = tmp_path/'worker.pid'
        python = tmp_path/'worker-python'
        python.write_text('#!' + sys.executable + '\nimport os,time\nfrom pathlib import Path\n'
                          + f'Path({str(pid_path)!r}).write_text(str(os.getpid()))\ntime.sleep(30)\n')
        python.chmod(0o700)
        service.profile['worker_python'] = str(python)
        directory = tmp_path/'request';directory.mkdir()
        task = asyncio.create_task(GraspSkills._worker(service, directory))
        deadline = time.monotonic()+3
        while not pid_path.exists():
            assert time.monotonic() < deadline
            await asyncio.sleep(.01)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        with pytest.raises(ProcessLookupError):
            os.kill(int(pid_path.read_text()), 0)
    asyncio.run(run())


@pytest.mark.parametrize('height', [0., -.3])
def test_current_lift_height_replaces_historical_scene_height(tmp_path, height):
    async def run():
        service, skills, backend, _ = make_service(tmp_path)
        backend.state['lift'] = height
        assert service.profile['scene']['lift_joint_m'] != height
        result = await make_obb_plan(skills)
        assert result['status'] == 'succeeded', result
        id = result['outputs']['plan_id']
        request = json.loads((service.directory / id / 'request.json').read_text())
        assert request['state']['lift'] == height
        result = await invoke(skills, 'execute_right_grasp', {'plan_id':id}, 'execute')
        assert result['status'] == 'succeeded', result
    asyncio.run(run())


@pytest.mark.parametrize('target', ['box cutter','banana','red cube','  blue block  ','方块'])
def test_open_vocabulary_targets_reach_worker(tmp_path, target):
    async def run():
        service, skills, backend, _ = make_service(tmp_path)
        declaration=next(s for s in skills.manifest()['skills'] if s['name']=='perceive_grasp_target')
        assert declaration['arguments']['target']=={'type':'string'}
        result=await invoke(skills,'perceive_grasp_target',{'observation_id':'image','target':target},'open')
        assert result['status']=='succeeded',result
        assert result['outputs']['target']==target.strip()
        request=json.loads((service.directory/result['outputs']['object_id']/'request.json').read_text())
        assert request['target']==target.strip()
        assert not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('target',['','   ','cube\x00','cube\n','x'*161])
def test_invalid_target_fails_before_compute(tmp_path,target):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        result=await invoke(skills,'perceive_grasp_target',{'observation_id':'image','target':target},'bad-target')
        assert result['status']=='failed'
        assert not list(service.directory.glob('*/request.json'))
        assert not backend.calls
    asyncio.run(run())


def test_worker_sets_detector_target_for_each_request(tmp_path,monkeypatch):
    import cv2
    from hei_agent_bridge.grasp import worker
    cv2.imwrite(str(tmp_path/'rgb.jpg'),np.zeros((3,3,3),dtype=np.uint8))
    np.save(tmp_path/'depth.npy',np.ones((3,3),dtype=np.float32))
    class Detector:
        def __init__(self): self.targets=[]
        def set_classes(self,classes): self.targets.append(classes)
        def predict(self,*args,**kwargs): return []
    detector=Detector()
    monkeypatch.setattr(worker,'camera_transform',lambda _: np.eye(4))
    request={'operation':'perceive','calibration':'unused','directory':str(tmp_path),'profile':{},'info':{'k':np.eye(3).ravel().tolist(),'d':[],
             'distortion_model':'plumb_bob'}}
    for target in ('cube','banana'):
        with pytest.raises(ValueError,match='No segmented RGB-D object'):
            worker.compute({**request,'target':target},detector=detector)
    assert detector.targets==[['cube'],['banana']]
