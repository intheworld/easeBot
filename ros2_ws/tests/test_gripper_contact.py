"""Load contact checks use distinct fresh measured feedback and never command current."""
import asyncio
from copy import deepcopy
from types import SimpleNamespace
import threading
import pytest
from hei_agent_bridge.grasp.contact import contact_evidence
from hei_agent_bridge.ros_skills import RosSkillBackend
from test_grasp_skills import make_service,invoke


def rows():return [(10+i*.02,-1.5,0.,.2,1.) for i in range(16)]
def evidence(data,now=10.3):
    return contact_evidence(data,now=now,closed_gap_m=0.,open_gap_m=.1,gap_per_rad=.1/4.5)

def test_blocked_loaded_jaw_confirms_contact():
    result=evidence(rows())
    assert result['physical_capture_verified'] and result['torque_nm']==pytest.approx(.2)

@pytest.mark.parametrize('case',['empty','unloaded','disabled','moving','stale','startup','spike','nan','open'])
def test_contact_rejects_unreliable_or_empty_feedback(case):
    data=[list(r) for r in rows()];now=10.3
    if case=='empty':
        for r in data:r[1]=0.
    if case=='unloaded':
        for r in data:r[3]=0.
    if case=='disabled':data[-1][4]=0.
    if case=='moving':data[-1][2]=.2
    if case=='stale':now=11.
    if case=='startup':data=data[-2:]
    if case=='spike':
        for r in data:r[3]=0.
        data[-1][3]=.5
    if case=='nan':data[-1][3]=float('nan')
    if case=='open':
        for r in data:r[1]=-4.5
    assert not evidence(data,now)['physical_capture_verified']


def test_ros_dynamic_uses_actual_effort_and_distinct_motor_sequences(monkeypatch):
    import hei_agent_bridge.ros_skills as module
    b=RosSkillBackend.__new__(RosSkillBackend);b.lock=threading.Lock()
    b._update=lambda *a:None
    clock=SimpleNamespace(now=10.)
    monkeypatch.setattr(module.time,'monotonic',lambda:clock.now)
    b.gripper_closure={'gap_m':0.,'started':9.}
    msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=0)),
        joint_names=['right_gripper_motor'],interface_values=[SimpleNamespace(
            interface_names=['position','velocity','effort','enabled','feedback_sequence','current_ratio'],
            values=[-1.5,0.,.2,1.,0.,.05])])
    for i in range(16):
        clock.now=10.+i*.02;msg.interface_values[0].values[4]=i
        b._dynamic(msg,'right');b._dynamic(msg,'right')
    assert len(b.gripper_contact_history)==16
    profile={'gap_per_motor_rad_m':-.1/4.5,'gripper_open_rad':-4.5}
    assert b.gripper_contact(profile)['physical_capture_verified']
    b.gripper_closure=None
    assert not b.gripper_contact(profile)['physical_capture_verified']


def test_observe_skill_can_report_contact_without_rgb_or_creating_payload(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        backend.gripper_contact=lambda profile:dict(physical_capture_verified=True,
            contact_state='sustained_jaw_load_contact',torque_nm=.2)
        async def capture(*args):raise RuntimeError('no image')
        service.cameras.capture=capture
        result=await invoke(skills,'observe_gripper',{'side':'right'},'contact')
        assert result['status']=='succeeded',result
        assert result['outputs']['physical_capture_verified']
        assert result['outputs']['torque_nm']==.2
        assert service.carry.payload is None and not backend.calls
    asyncio.run(run())


def test_contact_is_refreshed_after_camera_work(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        evidence={'physical_capture_verified':True,'contact_state':'sustained_jaw_load_contact','torque_nm':.2}
        backend.gripper_contact=lambda profile:deepcopy(evidence)
        async def capture(*args):
            evidence.update(physical_capture_verified=False,contact_state='feedback_unavailable',torque_nm=0.)
            return {'observation_id':'image'}
        service.cameras.capture=capture
        result=await invoke(skills,'observe_gripper_visual',{'side':'right'},'stale-after-image')
        assert not result['outputs']['physical_capture_verified']
        assert result['outputs']['holding_state']=='unknown'
    asyncio.run(run())


def test_default_contact_never_reads_camera_or_arm_scene(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        backend.gripper_position=lambda:-1.5
        backend.gripper_contact=lambda profile:dict(physical_capture_verified=True,
            contact_state='sustained_jaw_load_contact',torque_nm=.945)
        def forbidden(*args,**kwargs):raise AssertionError('Unexpected visual/arm scene access')
        backend.grasp_state=forbidden;service.cameras.capture=forbidden;service._worker=forbidden
        service.computing=True  # An independent visual job cannot delay the motor-only query.
        result=await invoke(skills,'observe_gripper',{'side':'right'},'motor-only')
        assert result['status']=='succeeded',result
        assert result['outputs']['physical_capture_verified']
        assert result['outputs']['wrist_state']=='not_requested'
        assert result['outputs']['observation_id']==''
        assert result['outputs']['holding_state']=='holding_contact_confirmed'
        assert not backend.calls
    asyncio.run(run())
