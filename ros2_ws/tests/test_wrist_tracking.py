"""Image registration regressions; no detector, ROS or robot motion."""
import json
from pathlib import Path
import cv2
import numpy as np
import pytest
from hei_agent_bridge.grasp.wrist_tracking import create_anchor,track_translation
from hei_agent_bridge.grasp.wrist import WristTargetError

FIXTURE=Path(__file__).parent/'fixtures/wrist_tracking_a1d7'


def replay_anchor(tmp_path):
    scene=json.loads((FIXTURE/'scene.json').read_text());frame=scene['frames'][0]
    image=cv2.imread(str(FIXTURE/'00.jpg'));mask=cv2.imread(str(FIXTURE/'anchor_mask.png'),0)
    anchor=create_anchor(image,mask,scene['calibration'],np.array(frame['camera']),frame['obb'],
                         np.array(scene['initial_delta_m']),tmp_path/'anchor.npz')
    return scene,anchor


def test_saved_mask_centroid_jump_does_not_move_tracked_target(tmp_path):
    scene,anchor=replay_anchor(tmp_path);centers=[];errors=[]
    for i,frame in enumerate(scene['frames'][:3]):
        report={};delta=track_translation(cv2.imread(str(FIXTURE/f'{i:02d}.jpg')),scene['calibration'],
            np.array(frame['camera']),frame['obb'],anchor,report)
        center=np.array(frame['obb']['center_m'])+delta;centers.append(center)
        up=np.array(frame['obb']['rotation'])[:,2]
        error=center+scene['hover_relative_to_object']-np.array(frame['tcp'])[:3,3]
        error-=up*(error@up);errors.append(np.linalg.norm(error))
        assert report['inliers']>=8 and report['registration_residual_p90_m']<=.002
    # The stationary target remains within the existing 2 mm alignment budget;
    # the third YOLO mask included shadow and reported a >7 mm correction.
    assert np.max(np.linalg.norm(np.array(centers)-centers[0],axis=1))<.002
    assert errors[2]<.002
    assert np.linalg.norm(scene['frames'][2]['old_alignment_error_m'])>.007
    assert errors[0]>errors[1]>errors[2]


def test_later_frame_can_track_without_falsely_certifying_convergence(tmp_path):
    from hei_agent_bridge.grasp.wrist import bounded_hover
    scene,anchor=replay_anchor(tmp_path);frame=scene['frames'][3];report={}
    delta=track_translation(cv2.imread(str(FIXTURE/'03.jpg')),scene['calibration'],
        np.array(frame['camera']),frame['obb'],anchor,report)
    center=np.array(frame['obb']['center_m'])+delta
    goal=np.array(frame['tcp']);goal[:3,3]=center+scene['hover_relative_to_object']
    _,converged,_=bounded_hover(np.array(frame['tcp']),np.array(scene['frames'][0]['tcp']),goal,
        np.array(frame['obb']['rotation'])[:,2],{'max_total_m':.02,'max_step_m':.005,'tolerance_m':.002})
    assert not converged
    assert report['metric_inlier_fraction']>=.6 and report['registration_residual_p90_m']<=.002


@pytest.mark.parametrize('kind',['blank','noise','covered'])
def test_lost_or_replaced_object_cannot_certify_alignment(tmp_path,kind):
    scene,anchor=replay_anchor(tmp_path);frame=scene['frames'][0]
    image=cv2.imread(str(FIXTURE/'00.jpg'))
    if kind=='blank':image[:]=100
    elif kind=='noise':image=np.random.default_rng(71).integers(0,256,image.shape,dtype=np.uint8)
    else:image[160:290,250:360]=0
    with pytest.raises(WristTargetError):
        track_translation(image,scene['calibration'],np.array(frame['camera']),frame['obb'],anchor,{})


def test_textureless_anchor_is_rejected(tmp_path):
    scene=json.loads((FIXTURE/'scene.json').read_text());frame=scene['frames'][0]
    with pytest.raises(WristTargetError,match='Insufficient object texture'):
        create_anchor(np.zeros((480,640,3),np.uint8),np.ones((480,640),np.uint8),
            scene['calibration'],np.array(frame['camera']),frame['obb'],np.zeros(3),tmp_path/'anchor.npz')


def test_calibration_change_does_not_reuse_anchor(tmp_path):
    scene,anchor=replay_anchor(tmp_path);frame=scene['frames'][0];scene['calibration']['k'][0]+=1
    with pytest.raises(WristTargetError,match='calibration changed'):
        track_translation(cv2.imread(str(FIXTURE/'00.jpg')),scene['calibration'],
                          np.array(frame['camera']),frame['obb'],anchor,{})


def test_real_pixel_translation_is_preserved_in_metric_correction(tmp_path):
    from hei_agent_bridge.grasp.wrist_tracking import plane_points
    scene,anchor=replay_anchor(tmp_path);frame=scene['frames'][0];camera=np.array(frame['camera'])
    image=cv2.imread(str(FIXTURE/'00.jpg'));shift=np.array([4.,-3.])
    moved=cv2.warpAffine(image,np.c_[np.eye(2),shift],(640,480))
    report={};delta=track_translation(moved,scene['calibration'],camera,frame['obb'],anchor,report)
    with np.load(anchor['anchor']) as data:
        points=data['pixels'];up=data['up'];height=float(data['height'])
        expected=np.array(scene['initial_delta_m'])+np.median(
            plane_points(points+shift,scene['calibration'],camera,up,height)-data['world'],axis=0)
    np.testing.assert_allclose(delta,expected,atol=.0001)


def test_worker_reuses_anchor_without_yolo_centroid(monkeypatch,tmp_path):
    from types import SimpleNamespace
    from hei_agent_bridge.grasp import wrist,planner,validation
    scene,anchor=replay_anchor(tmp_path);frame=scene['frames'][1]
    # Stub kinematics, path planning and collision work for this dispatch test;
    # registration runs on the saved pixels and calibrated camera transforms.
    # Build a self-contained request from fixture transforms.
    origin=scene['frames'][0];up=np.array(origin['obb']['rotation'])[:,2]
    pre=np.eye(4);pre[:3,:3]=np.array(origin['tcp'])[:3,:3]
    pre[:3,3]=np.array(origin['obb']['center_m'])+scene['hover_relative_to_object']
    goal=pre.copy();goal[:3,3]-=.08*up
    reference={'target_obb':frame['obb'],'T_pregrasp':pre.tolist(),'T_grasp':goal.tolist(),
               'wrist_tracking':anchor,'class_name':'blue block'}
    # Keep the original hover orientation and height while moving in its plane.
    current=np.array(frame['tcp'])
    monkeypatch.setattr(wrist,'refinement_config',lambda c:c)
    monkeypatch.setattr(wrist,'load_calibration',lambda p:scene['calibration'])
    monkeypatch.setattr('hei_agent_bridge.grasp.kinematics.RightArmFK',lambda p:lambda q:current)
    monkeypatch.setattr(planner,'RightArmPlanner',lambda p:SimpleNamespace(cartesian_path=lambda q,t:[np.zeros(6)]))
    monkeypatch.setattr(validation,'validate_plan',lambda *a:({'status':'passed'},None,None))
    cv2.imwrite(str(tmp_path/'rgb.jpg'),cv2.imread(str(FIXTURE/'01.jpg')))
    config={'calibration_file':'unused','max_total_m':.02,'max_step_m':.005,'tolerance_m':.002}
    req={'profile':{'wrist_refinement':config,'scene':{}},'directory':str(tmp_path),'urdf':'unused',
         'state':{'right':[0.]*6,'left':[0.]*6,'lift':0.,'left_gripper':0.,'right_gripper':-4.},
         'reference_plan':reference,'origin_plan':{'T_pregrasp':pre.tolist(),'target_obb':origin['obb']}}
    detector=SimpleNamespace(set_classes=lambda names:pytest.fail('tracking must not rerun YOLO'))
    output=wrist.refine_request(req,detector)
    assert output['reference_plan']['wrist_tracking']['anchor']==anchor['anchor']
    assert 'keyframe' in output['reference_plan']['wrist_tracking']
    assert 'keyframe' not in anchor  # Request evidence is not mutated.
    assert not output['converged']
    assert json.loads((tmp_path/'wrist.json').read_text())['method']=='fixed_anchor_lk_plane_registration'
    from hei_agent_bridge.grasp import wrist_tracking,wrist_keyframe
    original_track=wrist_tracking.track_translation
    def recovered(*args,**kwargs):
        delta=original_track(*args,**kwargs)
        args[5]['keyframe_recovery']=True
        return delta
    monkeypatch.setattr(wrist_tracking,'track_translation',recovered)
    monkeypatch.setattr(wrist_keyframe,'create_keyframe',lambda *a:pytest.fail('recovery must not seed another keyframe'))
    req['reference_plan']['wrist_tracking']=output['reference_plan']['wrist_tracking']
    recovered_output=wrist.refine_request(req,detector)
    assert recovered_output['reference_plan']['wrist_tracking']==req['reference_plan']['wrist_tracking']


def test_low_luminance_texture_saved_failure_uses_calibrated_metric_consensus(tmp_path):
    fixture=FIXTURE.parent/'wrist_tracking_6d2b';scene=json.loads((fixture/'scene.json').read_text());cal=scene['calibration']
    anchor=create_anchor(cv2.imread(str(fixture/'anchor.jpg')),cv2.imread(str(fixture/'mask.png'),0),
        cal,np.array(scene['anchor_camera']),scene['anchor_obb'],np.array(scene['initial_delta_m']),tmp_path/'anchor.npz')
    report={};delta=track_translation(cv2.imread(str(fixture/'current.jpg')),cal,
        np.array(scene['current_camera']),scene['current_obb'],anchor,report)
    assert report['registration_model']=='calibrated_planar_translation'
    assert report['flow_color_space']=='Lab'
    assert report['inliers']>=8 and report['metric_inlier_fraction']>=.6
    assert report['registration_residual_p90_m']<=.002 and report['patch_ncc_median']>=.8
    # A mask centroid jump is not needed to recover this observation.
    assert np.linalg.norm(delta)<.001


def test_saved_retries_still_reject_insufficient_metric_consensus(tmp_path):
    fixture=FIXTURE.parent/'wrist_tracking_6d2b';scene=json.loads((fixture/'scene.json').read_text());cal=scene['calibration']
    anchor=create_anchor(cv2.imread(str(fixture/'anchor.jpg')),cv2.imread(str(fixture/'mask.png'),0),
        cal,np.array(scene['anchor_camera']),scene['anchor_obb'],np.array(scene['initial_delta_m']),tmp_path/'anchor.npz')
    for retry in scene['retries']:
        report={}
        from hei_agent_bridge.grasp.wrist_tracking import _track_sparse_translation
        with pytest.raises(WristTargetError,match='inconsistent metric feature motion'):
            _track_sparse_translation(cv2.imread(str(fixture/retry['image'])),cal,np.array(retry['camera']),retry['obb'],anchor,report)
        assert report['metric_inlier_fraction']<.6
