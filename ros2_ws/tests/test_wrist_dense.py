"""Dense recovery has object-level evidence and explicit negative regressions."""
import json
from pathlib import Path
import cv2
import numpy as np
import pytest
from hei_agent_bridge.grasp.wrist_tracking import create_anchor, track_translation
from hei_agent_bridge.grasp.wrist_dense import dense_translation
from hei_agent_bridge.grasp.wrist import WristTargetError

FIXTURE=Path(__file__).parent/'fixtures/wrist_dense_fd29'


def anchor(tmp_path):
    scene=json.loads((FIXTURE/'scene.json').read_text())
    im=cv2.imread(str(FIXTURE/'anchor.jpg'));mask=cv2.imread(str(FIXTURE/'mask.png'),0)
    a=create_anchor(im,mask,scene['calibration'],np.array(scene['anchor_camera']),scene['anchor_obb'],
                    np.array(scene['initial_delta_m']),tmp_path/'anchor.npz')
    return scene,a,im,mask


def test_latest_sequence_recovers_lost_sparse_features_without_reanchoring(tmp_path):
    scene,a,_,_=anchor(tmp_path);centers=[];recovered=0
    for f in scene['frames']:
        image=cv2.imread(str(FIXTURE/f['image']));report={}
        delta=track_translation(image,scene['calibration'],np.array(f['camera']),f['obb'],a,report)
        if report['method']=='fixed_anchor_dense_registration':
            recovered+=1
            assert report['dense_score']>=.8 and report['reverse_score']>=.8
            assert report['reverse_error_m']<=.002
            assert report['color_luminance_difference_m']<=.002
            assert report['competing_peaks']==0
            assert 'sparse_attempt' in report
        centers.append(np.array(f['obb']['center_m'])+delta)
    assert recovered==3  # All three actual retries lost sparse texture.
    assert np.ptp(np.array(centers),axis=0).max()<.002


@pytest.mark.parametrize('kind',['blank','noise','removed','wrong_color','covered','duplicate'])
def test_absent_changed_occluded_and_ambiguous_targets_are_not_accepted(tmp_path,kind):
    scene,a,image,mask=anchor(tmp_path)
    if kind=='blank':image[:]=100
    elif kind=='noise':image=np.random.default_rng(37).integers(0,256,image.shape,dtype=np.uint8)
    elif kind=='wrong_color':image=image[:,:,::-1].copy()
    elif kind=='removed':image=cv2.inpaint(image,cv2.dilate(mask,np.ones((15,15),np.uint8)),5,cv2.INPAINT_TELEA)
    elif kind=='covered':image[230:310,240:350]=0
    else:
        x,y,w,h=cv2.boundingRect(mask);pad=6;x-=pad;y-=pad;w+=2*pad;h+=2*pad
        patch=image[y:y+h,x:x+w].copy()
        image=cv2.inpaint(image,cv2.dilate(mask,np.ones((15,15),np.uint8)),5,cv2.INPAINT_TELEA)
        for dx in (-48,48):image[y:y+h,x+dx:x+dx+w]=patch
    report={}
    with pytest.raises(WristTargetError):
        dense_translation(image,scene['calibration'],np.array(scene['anchor_camera']),scene['anchor_obb'],a,report,.02)
    if kind=='duplicate':assert report.get('competing_peaks',0)>0


def test_no_dense_evidence_in_old_anchor_does_not_force_a_match(tmp_path):
    scene,a,image,_=anchor(tmp_path)
    with np.load(a['anchor']) as data:old={k:data[k] for k in data if k not in ('image','mask','camera')}
    np.savez_compressed(a['anchor'],**old)
    with pytest.raises(WristTargetError,match='no dense image evidence'):
        dense_translation(image,scene['calibration'],np.array(scene['anchor_camera']),scene['anchor_obb'],a,{},.02)


def test_one_saturated_channel_does_not_discard_other_image_evidence(tmp_path):
    scene,_,image,mask=anchor(tmp_path);image[:,:,2]=255
    a=create_anchor(image,mask,scene['calibration'],np.array(scene['anchor_camera']),scene['anchor_obb'],
                    np.array(scene['initial_delta_m']),tmp_path/'saturated.npz')
    report={}
    delta=dense_translation(image,scene['calibration'],np.array(scene['anchor_camera']),scene['anchor_obb'],a,report,.02)
    assert report['dense_score']>.99
    np.testing.assert_allclose(delta,scene['initial_delta_m'],atol=1e-8)
