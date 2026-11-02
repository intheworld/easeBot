"""Current-session geometry and sensor evidence, with no physical robot calls."""
import asyncio
import json
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace
import cv2
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/hei_agent_bridge/src'))
from test_grasp_skills import make_service,invoke


def test_wrist_without_current_model_returns_image_without_inventing_payload(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        async def capture(*args):return {'observation_id':'wrist-image'}
        service.cameras.capture=capture
        result=await invoke(skills,'observe_gripper_visual',{'side':'right'},'observe')
        assert result['status']=='succeeded',result
        output=result['outputs']
        assert output['wrist_state']=='no_current_model' and not output['model_present']
        assert output['holding_state']=='unknown'
        assert output['observation_id']=='wrist-image'
        assert not output['physical_capture_verified'] and not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('visible',[True,False])
def test_known_model_visual_observation_is_read_only_and_not_capture_proof(tmp_path,monkeypatch,visible):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        service.profile['wrist_refinement']['calibration_file']='measured-calibration'
        service.carry.attach(dict(center_m=[0,0,0],rotation=np.eye(3).tolist(),size_m=[.03]*3,target='cube'),
                             np.eye(4),backend.state['right_gripper'])
        token=service.carry.token
        async def acquire(*args):return {'observation_id':'fresh-wrist'},{'stamp_sec':10.},deepcopy(backend.state)
        monkeypatch.setattr('hei_agent_bridge.grasp.capture.acquire',acquire)
        async def worker(directory):
            request=json.loads((directory/'request.json').read_text())
            assert request['operation']=='observe_payload' and request['payload']['detection_prompt']=='cube'
            return dict(wrist_state='target_consistent_with_payload' if visible else 'inconclusive',object_visible=visible)
        service._worker=worker
        result=await invoke(skills,'observe_gripper_visual',{'side':'right'},'observe')
        assert result['status']=='succeeded',result
        assert result['outputs']['object_visible']==visible
        assert result['outputs']['holding_state']==('holding_candidate' if visible else 'unknown')
        assert service.carry.token==token and service.carry.status=='attached'
        assert not result['outputs']['physical_capture_verified'] and not backend.calls
    asyncio.run(run())


@pytest.mark.parametrize('visible',[True,False])
def test_offline_wrist_mask_missing_is_unknown_never_empty(tmp_path,monkeypatch,visible):
    from hei_agent_bridge.grasp.wrist import WristTargetError
    from hei_agent_bridge.grasp.worker import compute
    cv2.imwrite(str(tmp_path/'rgb.jpg'),np.zeros((10,10,3),np.uint8))
    monkeypatch.setattr('hei_agent_bridge.grasp.wrist.load_calibration',lambda _:dict(height=10,width=10))
    monkeypatch.setattr('hei_agent_bridge.grasp.kinematics.RightArmFK',lambda _:lambda q:np.eye(4))
    def detect(*args,**kwargs):
        if not visible:raise WristTargetError(0)
        return np.zeros(3),np.ones((10,10),bool),None
    monkeypatch.setattr('hei_agent_bridge.grasp.wrist_detection.detect_translation',detect)
    d=SimpleNamespace(set_classes=lambda names:None)
    request=dict(operation='observe_payload',directory=str(tmp_path),urdf='model',state={'right':[0.]*6},
        profile={'wrist_refinement':{'calibration_file':'calibration','max_total_m':.02}},
        payload={'detection_prompt':'cube','T_tcp_object':np.eye(4).tolist(),'size_m':[.03]*3})
    result=compute(request,d)
    assert result['object_visible']==visible
    assert result['wrist_state']==('target_consistent_with_payload' if visible else 'inconclusive')
    assert (tmp_path/'holding.json').exists()


def test_fully_open_feedback_recovers_interrupted_task_context(tmp_path):
    service,_,backend,_=make_service(tmp_path)
    service.carry.attach(dict(center_m=[0,0,0],rotation=np.eye(3).tolist(),size_m=[.03]*3),
                         np.eye(4),backend.state['right_gripper'],pending=True)
    assert service.carry.status=='unknown'
    backend.state['right_gripper']=service.profile['gripper_open_rad']
    assert service.carry.check(backend.state)['status']=='empty'
    service.carry.require_empty()


def test_visual_result_discarded_when_arm_moves_during_compute(tmp_path,monkeypatch):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        service.profile['wrist_refinement']['calibration_file']='calibration'
        service.carry.attach(dict(center_m=[0,0,0],rotation=np.eye(3).tolist(),size_m=[.03]*3,target='cube'),
                             np.eye(4),backend.state['right_gripper'])
        async def acquire(*args):return {'observation_id':'image'},{'stamp_sec':10.},deepcopy(backend.state)
        monkeypatch.setattr('hei_agent_bridge.grasp.capture.acquire',acquire)
        async def worker(directory):
            backend.state['right'][0]+=.01
            return dict(wrist_state='target_consistent_with_payload',object_visible=True)
        service._worker=worker
        result=await invoke(skills,'observe_gripper_visual',{'side':'right'},'observe-moving')
        assert result['status']=='succeeded',result
        assert result['outputs']['wrist_state']=='scene_changed'
        assert not result['outputs']['object_visible'] and result['outputs']['holding_state']=='unknown'
        assert not backend.calls
    asyncio.run(run())


def test_camera_missing_still_returns_opening_without_claiming_empty(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        async def capture(*args):raise RuntimeError('no fresh RGB')
        service.cameras.capture=capture
        result=await invoke(skills,'observe_gripper_visual',{'side':'right'},'no-camera')
        assert result['status']=='succeeded',result
        assert result['outputs']['wrist_state']=='camera_unavailable'
        assert result['outputs']['gap_m']>=0 and result['outputs']['holding_state']=='unknown'
        assert not backend.calls
    asyncio.run(run())
