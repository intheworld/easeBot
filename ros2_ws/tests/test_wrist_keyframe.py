"""Saved late-view recovery without recursive template drift or relaxed gates."""
import json
from pathlib import Path
import cv2
import numpy as np
import pytest
from hei_agent_bridge.grasp.wrist_tracking import create_anchor,track_translation
from hei_agent_bridge.grasp.wrist_keyframe import create_keyframe
from hei_agent_bridge.grasp.wrist import WristTargetError

FIXTURE=Path(__file__).parent/'fixtures/wrist_keyframe_8475'


def evidence(tmp_path):
    s=json.loads((FIXTURE/'scene.json').read_text());f=s['frames'][0]
    a=create_anchor(cv2.imread(str(FIXTURE/'00.jpg')),cv2.imread(str(FIXTURE/'mask.png'),0),s['calibration'],
        np.array(f['camera']),f['obb'],np.array(s['initial_delta_m']),tmp_path/'original.npz')
    f=s['frames'][2];im=cv2.imread(str(FIXTURE/'02.jpg'));report={}
    delta=track_translation(im,s['calibration'],np.array(f['camera']),f['obb'],a,report)
    a['keyframe']=create_keyframe(im,s['calibration'],np.array(f['camera']),f['obb'],delta,a,tmp_path/'keyframe.npz')
    return s,a


def test_saved_final_observation_recovers_with_original_bound_center(tmp_path):
    s,a=evidence(tmp_path);f=s['frames'][3];im=cv2.imread(str(FIXTURE/'03.jpg'));report={}
    with pytest.raises(WristTargetError):track_translation(im,s['calibration'],np.array(f['camera']),f['obb'],{k:v for k,v in a.items() if k!='keyframe'}, {})
    delta=track_translation(im,s['calibration'],np.array(f['camera']),f['obb'],a,report)
    assert report['keyframe_recovery'] and report['dense_score']>.93
    assert report['reverse_error_m']<=.002 and report['color_luminance_difference_m']<=.002
    error=np.array(f['pregrasp'])[:3,3]+delta-np.array(f['tcp'])[:3,3]
    up=np.array(f['obb']['rotation'])[:,2];error-=up*(error@up)
    assert np.linalg.norm(error)<.002
    # Absolute center inherited from direct original registration, not a mask centroid.
    np.testing.assert_allclose(np.array(f['obb']['center_m'])+delta,[.4659026,.0889372,-.0874085],atol=1e-6)


@pytest.mark.parametrize('kind',['blank','noise','removed','wrong_color','duplicate'])
def test_keyframe_does_not_accept_absent_replaced_or_ambiguous_target(tmp_path,kind):
    s,a=evidence(tmp_path);f=s['frames'][2];im=cv2.imread(str(FIXTURE/'02.jpg'))
    with np.load(a['keyframe']['anchor']) as d:mask=d['mask']*255
    if kind=='blank':im[:]=100
    elif kind=='noise':im=np.random.default_rng(37).integers(0,256,im.shape,dtype=np.uint8)
    elif kind=='wrong_color':im=im[:,:,::-1].copy()
    else:
        original=im.copy();im=cv2.inpaint(im,cv2.dilate(mask,np.ones((15,15),np.uint8)),5,cv2.INPAINT_TELEA)
        if kind=='duplicate':
            x,y,w,h=cv2.boundingRect(mask);x-=6;y-=6;w+=12;h+=12
            for dx in (-48,48):im[y:y+h,x+dx:x+dx+w]=original[y:y+h,x:x+w]
    with pytest.raises(WristTargetError):track_translation(im,s['calibration'],np.array(f['camera']),f['obb'],a,{})
