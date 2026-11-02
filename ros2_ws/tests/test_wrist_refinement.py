"""Wrist RGB geometry and grasp-internal bounded refinement; no hardware."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/hei_agent_bridge/src'))
from hei_agent_bridge.grasp.wrist import (load_calibration,refinement_config,estimate_translation,
                                         projected_silhouette,refine_request,frame_quality)
from test_grasp_skills import make_service,make_obb_plan,invoke


def calibration():
    return dict(schema='hei.wrist_camera/v1',camera='right_wrist',frame_id='right_wrist_camera_optical_frame',
                tool_frame='a_right_tcp',width=640,height=480,k=[500.,0.,320.,0.,500.,240.,0.,0.,1.],
                d=[0.]*5,distortion_model='plumb_bob',
                T_tcp_camera=np.diag([1.,-1.,-1.,1.]).tolist())


def write_calibration(tmp_path):
    p=tmp_path/'wrist.json';p.write_text(json.dumps(calibration()));return str(p)


def obb():
    return dict(center_m=[0.,0.,0.],size_m=[.04,.04,.04],rotation=np.eye(3).tolist())


def detected(mask):
    return SimpleNamespace(names={0:'cube'},boxes=SimpleNamespace(cls=np.array([0]),conf=np.array([.9])),
                           masks=SimpleNamespace(data=mask[None].astype(float)))


def box_mask(center=(320,240)):
    mask=np.zeros((480,640),dtype=np.uint8)
    cv2.rectangle(mask,(center[0]-26,center[1]-26),(center[0]+26,center[1]+26),1,-1)
    return mask


def test_measured_calibration_and_config_validation(tmp_path):
    assert load_calibration(write_calibration(tmp_path))['camera']=='right_wrist'
    with pytest.raises(ValueError,match='measured'):load_calibration('')
    for key,value in [('frame_id','wrong'),('width',None),('k',[]),('T_tcp_camera',np.zeros((4,4)).tolist())]:
        data=calibration();data[key]=value;p=tmp_path/'bad.json';p.write_text(json.dumps(data))
        with pytest.raises((ValueError,TypeError)):load_calibration(p)
    for value in [{'enabled':'yes'},{'max_updates':True},{'max_step_m':.04},{'tolerance_m':.005,'max_step_m':.001}]:
        with pytest.raises(ValueError):refinement_config(value)


def test_zero_error_and_signed_metric_correction():
    pose=np.eye(4);pose[2,3]=.4;c=calibration()
    delta,_,_=estimate_translation(obb(),pose,c,[detected(box_mask())],'cube',.02)
    np.testing.assert_allclose(delta,0,atol=1e-8)
    delta,_,_=estimate_translation(obb(),pose,c,[detected(box_mask((332,240)))],'cube',.02)
    # At the top plane .38m from camera, 12 pixels -> +9.12mm.
    np.testing.assert_allclose(delta,[.00912,0,0],atol=1e-6)
    delta,_,_=estimate_translation(obb(),pose,c,[detected(box_mask((320,252)))],'cube',.02)
    np.testing.assert_allclose(delta,[0,-.00912,0],atol=1e-6)


@pytest.mark.parametrize('case',['missing','ambiguous','large','clipped','wrong_mask_resolution'])
def test_untrusted_observations_rejected(case):
    pose=np.eye(4);pose[2,3]=.4;rs=[detected(box_mask())]
    if case=='missing':rs=[]
    if case=='ambiguous':rs*=2
    if case=='large':rs=[detected(box_mask((380,240)))]
    if case=='clipped':rs=[detected(box_mask((5,240)))]
    if case=='wrong_mask_resolution':rs=[detected(np.ones((240,320),dtype=np.uint8))]
    with pytest.raises(ValueError):estimate_translation(obb(),pose,calibration(),rs,'cube',.02)


def enable_wrist(service,cameras,tmp_path):
    config=refinement_config({'enabled':True,'calibration_file':write_calibration(tmp_path),'settle_sec':.1,'joint_stable_sec':.1})
    service.profile['wrist_refinement']=config
    # Tests intentionally update binding after installing this synthetic calibration.
    service.binding=service._binding()
    cameras.cache=SimpleNamespace(frame=lambda name:SimpleNamespace(frame_id='right_wrist_camera_optical_frame',width=640,height=480))
    original=cameras._get;images={}
    async def capture(args,invocation):
        key=str(len(images));images[key]={'simulated':False,'received':time.monotonic(),
            'stamp_sec':time.time(),'frame_id':'right_wrist_camera_optical_frame',
            'assets':{'rgb':('image/jpeg',cv2.imencode('.jpg',np.random.default_rng(7).integers(0,256,(480,640,3),dtype=np.uint8))[1].tobytes())}}
        return {'observation_id':key}
    cameras.capture=capture
    cameras._get=lambda key:images[key] if key in images else original(key)
    return images


def test_missing_calibration_rejects_before_motion(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        service.profile['wrist_refinement']['enabled']=True
        service.binding=service._binding()
        result=await make_obb_plan(skills);id=result['outputs']['plan_id']
        result=await invoke(skills,'execute_right_grasp',{'plan_id':id},'exec')
        assert result['status']=='failed' and 'measured' in result['message']
        assert not backend.calls and not service.plans[id]['consumed']
    asyncio.run(run())


def test_refinement_is_inside_grasp_and_reobserves_before_descent(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        result=await make_obb_plan(skills);id=result['outputs']['plan_id']
        original=service._worker;calls=[]
        async def compute(directory):
            request=json.loads((directory/'request.json').read_text())
            if request['operation']!='refine_wrist':return await original(directory)
            calls.append(request)
            plan=deepcopy(request['reference_plan']);plan['pregrasp_joint_path']=[[.15]*6]
            return dict(plan=plan,reference_plan=plan,validation={'status':'passed'},converged=len(calls)==2)
        service._worker=compute
        result=await invoke(skills,'execute_right_grasp',{'plan_id':id},'exec')
        assert result['status']=='succeeded',result
        assert result['outputs']['wrist_refined'] is True
        assert len(images)==8 and len(calls)==2
        assert [v[0] for v in backend.calls]==['open','trajectory','trajectory','trajectory','close']
        assert backend.calls[2][2]==[[.15]*6]
    asyncio.run(run())


@pytest.mark.parametrize('problem',['collision','nonconvergence','scene','cancellation','timeout','drift','binding','stale'])
def test_failed_refinement_never_descends_or_closes(tmp_path,problem):
    async def run():
        service,skills,backend,_=make_service(tmp_path);images=enable_wrist(service,service.cameras,tmp_path)
        service.profile['wrist_refinement']['max_updates']=1
        if problem=='stale':
            async def stale(args,invocation):
                images['old']={'simulated':False,'received':0,'stamp_sec':0}
                return {'observation_id':'old'}
            service.cameras.capture=stale;service.profile['wrist_refinement']['frame_timeout_sec']=.01
        service.binding=service._binding()
        result=await make_obb_plan(skills);id=result['outputs']['plan_id']
        async def compute(directory):
            request=json.loads((directory/'request.json').read_text())
            if problem=='cancellation':raise asyncio.CancelledError()
            if problem=='timeout':raise asyncio.TimeoutError('compute timeout')
            if problem=='scene':backend.state['lift']+=.01
            if problem=='drift':backend.state['right'][0]+=.01
            if problem=='binding':service.profile['joint_speed_rad_s']=.09
            plan=deepcopy(request['reference_plan']);plan['pregrasp_joint_path']=[[.15]*6]
            return dict(plan=plan,reference_plan=plan,validation={'status':'failed' if problem=='collision' else 'passed'},
                        converged=problem!='nonconvergence')
        service._worker=compute
        result=await invoke(skills,'execute_right_grasp',{'plan_id':id},'exec')
        assert result['status']!='succeeded',result
        assert not any(v[0]=='close' for v in backend.calls)
        assert len(backend.calls)==(3 if problem=='nonconvergence' else 2)
        assert service.plans[id]['consumed']
    asyncio.run(run())


@pytest.mark.parametrize('pixel_shift',[0,20])
def test_worker_checks_real_ik_and_every_hover_sample(tmp_path,pixel_shift):
    pytest.importorskip('mujoco')
    from hei_agent_bridge.grasp.planner import RightArmPlanner
    from hei_agent_bridge.grasp.obb import support_geometry
    from hei_agent_bridge.grasp.validation import trajectory_samples
    urdf=ROOT/'ros2_ws/src/hei_description/urdf/hei_body.urdf'
    profile=json.loads((ROOT/'ros2_ws/src/hei_agent_bridge/config/right_grasp.json').read_text())
    q=np.array([.0556,-2.383713,-1.927085,1.114171,0.,.036688])
    planner=RightArmPlanner(urdf);pre=planner.fk(q)
    state=dict(right=q.tolist(),right_gripper=-4.,left=[0.]*6,left_gripper=0.,lift=0.)
    up,_=support_geometry(urdf,state,profile['scene'])
    goal=pre.copy();goal[:3,3]-=up*.08
    axis=np.array([1.,0.,0.]);axis-=up*(axis@up);axis/=np.linalg.norm(axis)
    target=dict(schema='hei.obb/v1',frame_id='a_right_base_link',center_m=(goal[:3,3]-up*.005).tolist(),
                size_m=[.04,.04,.03],rotation=np.column_stack((axis,np.cross(up,axis),up)).tolist())
    base_camera=np.diag([1.,-1.,-1.,1.]);base_camera[:3,3]=pre[:3,3]+up*.1
    c=calibration();c['T_tcp_camera']=(np.linalg.inv(pre)@base_camera).tolist()
    p=tmp_path/'calibration.json';p.write_text(json.dumps(c))
    profile['wrist_refinement']=refinement_config({'enabled':True,'calibration_file':str(p)})
    plan=dict(schema_version=3,strategy='top_down_direct_ik',class_name='cube',target_obb=target,
              urdf_sha256=planner.fk.sha256,base_frame='a_right_base_link',tool_frame='a_right_tcp',
              T_pregrasp=pre.tolist(),T_grasp=goal.tolist(),motion_config={k:profile[k] for k in
              ('joint_speed_rad_s','gripper_open_rad','gripper_closed_rad')})
    expected,area=projected_silhouette(target,base_camera,c)
    half=int(round(np.sqrt(area)/2))
    mask=np.zeros((480,640),dtype=np.uint8);x,y=np.rint(expected).astype(int)
    cv2.rectangle(mask,(x-half+pixel_shift,y-half),(x+half+pixel_shift,y+half),1,-1)
    detector=SimpleNamespace(set_classes=lambda names:None,predict=lambda *a,**k:[detected(mask)])
    image=np.zeros((480,640,3),dtype=np.uint8)
    image[mask>0]=np.random.default_rng(27).integers(30,230,(int(mask.sum()),3),dtype=np.uint8)
    cv2.imwrite(str(tmp_path/'rgb.jpg'),image)
    request=dict(profile=profile,urdf=str(urdf),state=state,directory=str(tmp_path),reference_plan=plan,origin_plan=plan)
    result=refine_request(request,detector)
    assert result['converged']==(pixel_shift==0)
    assert result['validation']['status']=='passed',result['validation']['errors']
    new_pre=np.array(result['plan']['T_pregrasp'])
    step=new_pre[:3,3]-pre[:3,3]
    assert np.linalg.norm(step)==pytest.approx(.005 if pixel_shift else 0.,abs=1e-7)
    if not pixel_shift:
        final=np.array(result['plan']['T_grasp'])
        delta=final[:3,3]-new_pre[:3,3]
        assert np.linalg.norm(delta-up*(delta@up))<1e-6
    assert abs(step@up)<1e-8
    np.testing.assert_allclose(planner.fk(result['plan']['pregrasp_joint_rad']),new_pre,atol=1e-4)
    assert 'pregrasp_joint_path' in result['plan']
    # Every generated waypoint is screened, not only the final correction pose.
    assert len(list(trajectory_samples(result['plan'])))>10


def test_pregrasp_path_endpoint_must_match():
    from hei_agent_bridge.grasp.validation import trajectory_samples
    plan=dict(motion_config=dict(joint_speed_rad_s=.1,gripper_open_rad=-4.,gripper_closed_rad=0.),
              start_joint_rad=[0.]*6,start_gripper_rad=-4.,pregrasp_joint_rad=[.1]*6,
              pregrasp_joint_path=[[.2]*6],approach_joint_rad=[[.3]*6])
    with pytest.raises(ValueError,match='endpoint'):list(trajectory_samples(plan))


def test_measured_calibration_is_bound_to_plan_revision(tmp_path):
    from hei_agent_bridge.grasp.service import GraspSkills
    service,_,backend,_=make_service(tmp_path)
    profile=deepcopy(service.profile)
    path=write_calibration(tmp_path)
    profile['wrist_refinement']=refinement_config({'enabled':True,'calibration_file':path})
    bound=GraspSkills(profile,service.cameras,backend,directory=tmp_path/'bound',urdf=service.urdf)
    assert str(Path(path).resolve()) in bound.extra_files
    before=bound.binding
    data=calibration();data['k'][0]+=1;Path(path).write_text(json.dumps(data))
    assert bound._binding()!=before


def test_frame_quality_rejects_blur_and_reports_motion():
    image=np.random.default_rng(4).integers(0,256,(480,640),dtype=np.uint8)
    encode=lambda im:cv2.imencode('.jpg',im)[1].tobytes()
    previous,first=frame_quality(encode(image))
    _,same=frame_quality(encode(image),previous)
    _,moved=frame_quality(encode(255-image),previous)
    _,blur=frame_quality(encode(cv2.GaussianBlur(image,(31,31),8)))
    assert first['sharpness']>40 and first['frame_difference'] is None
    assert same['frame_difference']==0 and moved['frame_difference']>3
    assert blur['sharpness']<40
    with pytest.raises(ValueError,match='decode'):frame_quality(b'not jpeg')


@pytest.mark.parametrize('problem',['blur','motion','duplicate'])
def test_bad_frames_never_reach_worker_or_descent(tmp_path,problem):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        images=enable_wrist(service,service.cameras,tmp_path)
        capture=service.cameras.capture;duplicate_stamp=[]
        async def blurry(args,invocation):
            result=await capture(args,invocation)
            image=images[result['observation_id']]
            if problem=='blur':
                image['assets']['rgb']=('image/jpeg',cv2.imencode('.jpg',np.full((480,640,3),100,dtype=np.uint8))[1].tobytes())
            elif problem=='motion':
                frame=np.random.default_rng(len(images)).integers(0,256,(480,640,3),dtype=np.uint8)
                image['assets']['rgb']=('image/jpeg',cv2.imencode('.jpg',frame)[1].tobytes())
            else:
                if len(images)==1:duplicate_stamp.append(image['stamp_sec'])
                image['stamp_sec']=duplicate_stamp[0]
            return result
        service.cameras.capture=blurry
        service.profile['wrist_refinement']['frame_timeout_sec']=.15
        # A sharpness gate is opt-in and must be calibrated for the actual scene.
        if problem=='blur':service.profile['wrist_refinement']['min_sharpness']=40.
        service.binding=service._binding()
        plan=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':plan['outputs']['plan_id']},'blur')
        assert result['status']=='failed' and 'stable fresh' in result['message']
        assert [v[0] for v in backend.calls]==['open','trajectory']
        logs=list((tmp_path/'plans').glob('*/wrist_00/capture_quality.json'))
        assert logs
        rows=json.loads(logs[0].read_text())
        assert rows and all(row['stable_count']==0 for row in rows)
        if problem=='blur':assert rows[-1]['sharpness']==0
        if problem=='duplicate':assert len(rows)==1
    asyncio.run(run())


def test_default_quality_gate_accepts_stable_low_texture_frames(tmp_path):
    async def run():
        service,skills,backend,_=make_service(tmp_path)
        images=enable_wrist(service,service.cameras,tmp_path)
        assert service.profile['wrist_refinement']['min_sharpness']==0
        capture=service.cameras.capture
        async def low_texture(args,invocation):
            result=await capture(args,invocation)
            images[result['observation_id']]['assets']['rgb']=('image/jpeg',cv2.imencode('.jpg',np.full((480,640,3),100,dtype=np.uint8))[1].tobytes())
            return result
        service.cameras.capture=low_texture
        original=service._worker
        async def compute(directory):
            request=json.loads((directory/'request.json').read_text())
            if request['operation']!='refine_wrist':return await original(directory)
            plan=deepcopy(request['reference_plan'])
            return dict(plan=plan,reference_plan=plan,validation={'status':'passed'},converged=True)
        service._worker=compute
        plan=await make_obb_plan(skills)
        result=await invoke(skills,'execute_right_grasp',{'plan_id':plan['outputs']['plan_id']},'low-texture')
        assert result['status']=='succeeded',result
        assert result['outputs']['wrist_refined']
    asyncio.run(run())


def test_total_target_estimate_saturates_without_immediate_failure():
    from hei_agent_bridge.grasp.wrist import bounded_hover
    config=refinement_config({});origin=np.eye(4);current=origin.copy();current[0,3]=.005
    estimated=origin.copy();estimated[:2,3]=[.014749167208437663,.014373092099325496]
    pre,converged,report=bounded_hover(current,origin,estimated,np.array([0.,0.,1.]),config)
    assert report['target_offset_saturated'] and not converged
    assert np.linalg.norm(report['limited_hover_offset_m'])==pytest.approx(.02)
    assert np.linalg.norm(pre[:3,3]-current[:3,3])==pytest.approx(.005)
    assert np.linalg.norm(report['hover_offset_m'])<=.02+1e-12
    assert pre[2,3]==current[2,3]


def test_saturated_boundary_is_not_false_visual_convergence():
    from hei_agent_bridge.grasp.wrist import bounded_hover
    origin=np.eye(4);current=origin.copy();current[0,3]=.02;estimated=origin.copy();estimated[0,3]=.025
    pre,converged,report=bounded_hover(current,origin,estimated,np.array([0.,0.,1.]),refinement_config({}))
    assert not converged and report['target_offset_saturated']
    np.testing.assert_allclose(pre,current)
    assert np.linalg.norm(report['alignment_error_m'])==pytest.approx(.005)
    estimated[0,3]=.02059
    _,converged,_=bounded_hover(current,origin,estimated,np.array([0.,0.,1.]),refinement_config({}))
    assert converged  # Actual residual .59mm is within the unchanged2mm tolerance.


def test_small_hover_overshoot_is_clipped_but_recovery_step_stays_bounded():
    from hei_agent_bridge.grasp.wrist import bounded_hover
    origin=np.eye(4);current=origin.copy();current[0,3]=.0205;estimated=current.copy()
    pre,converged,report=bounded_hover(current,origin,estimated,np.array([0.,0.,1.]),refinement_config({}))
    assert not converged
    assert pre[0,3]==pytest.approx(.02)
    assert np.linalg.norm(report['applied_step_m'])<=.005


@pytest.mark.parametrize('settles',[True,False])
def test_late_visual_recovery_gets_bounded_correction_before_final_recheck(tmp_path,settles):
    """The recorded fourth observation is valid but 2.452 mm is not converged."""
    async def run():
        from hei_agent_bridge.grasp.wrist import bounded_hover
        service,skills,backend,_=make_service(tmp_path)
        enable_wrist(service,service.cameras,tmp_path)
        assert service.profile['wrist_refinement']['max_updates']==5
        service.profile['wrist_refinement']['tolerance_m']=.002  # Historical strict policy exercised by this regression.
        service.binding=service._binding()
        result=await make_obb_plan(skills);plan_id=result['outputs']['plan_id'];calls=[]
        # Recorded alignment errors from plan 9eaa38d0. The fifth is a synthetic
        # successful feedback observation, NOT a prediction of physical success.
        errors=[[.008723995,.012100282,0],[.006120356,.009244972,0],
                [.003124209,.005346258,0],[.000652122,.002363228,0]]
        async def compute(directory):
            request=json.loads((directory/'request.json').read_text());i=len(calls);calls.append(request)
            error=np.array(errors[min(i,3)]) if i<4 or not settles else np.array([.0005,.001,0])
            current=np.eye(4);estimated=current.copy();estimated[:3,3]=error
            _,converged,diag=bounded_hover(current,current,estimated,np.array([0,0,1]),service.profile['wrist_refinement'])
            (directory/'wrist.json').write_text(json.dumps(diag))
            assert np.linalg.norm(diag['applied_step_m'])<=.005+1e-12
            plan=deepcopy(request['reference_plan']);plan['pregrasp_joint_path']=[[.15]*6]
            return dict(plan=plan,reference_plan=plan,validation={'status':'passed'},converged=converged)
        service._worker=compute
        result=await invoke(skills,'execute_right_grasp',{'plan_id':plan_id},'exec')
        assert result['status']==('succeeded' if settles else 'failed'),result
        assert len(calls)==(5 if settles else 6)
        terminal=json.loads((service.directory/plan_id/f'wrist_{len(calls)-1:02d}/refinement_status.json').read_text())
        assert terminal['commands_executed']==(4 if settles else 5)
        assert not terminal['next_correction_planned']
        assert terminal['budget_exhausted']==(not settles)
        fourth=json.loads((service.directory/plan_id/'wrist_03/refinement_status.json').read_text())
        assert fourth['remaining_error_m']>.002 and not fourth['converged']
        assert fourth['next_correction_planned']
        if settles:assert backend.calls[-1][0]=='close'
        else:
            assert 'remaining error 2.452 mm' in result['message']
            assert not any(c[0]=='close' for c in backend.calls)
    asyncio.run(run())


def test_default_five_mm_policy_accepts_latest_valid_residual():
    from hei_agent_bridge.grasp.wrist import bounded_hover
    config=refinement_config({})
    assert config['tolerance_m']==.005 and config['max_step_m']==.005
    current=np.eye(4);goal=current.copy();goal[:3,3]=[.00065212169,.00236322799,0.]
    _,converged,diagnostics=bounded_hover(current,current,goal,np.array([0,0,1]),config)
    assert converged
    np.testing.assert_allclose(diagnostics['applied_step_m'],0.)
    goal[0,3]=.006
    assert not bounded_hover(current,current,goal,np.array([0,0,1]),config)[1]
