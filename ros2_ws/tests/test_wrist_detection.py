"""Calibrated ROI recovery preserves geometry, confidence and ambiguity gates."""
from types import SimpleNamespace

import numpy as np
import pytest

from test_wrist_refinement import calibration, obb, box_mask
from hei_agent_bridge.grasp.wrist import WristTargetError
from hei_agent_bridge.grasp.wrist_detection import mapped_instances,search_roi,detect_translation


def results(masks,scores=None):
    scores=scores or [.9]*len(masks)
    return [SimpleNamespace(names={0:'cube'},boxes=SimpleNamespace(cls=np.zeros(len(masks)),conf=np.array(scores)),
                            masks=SimpleNamespace(data=np.array(masks)))]


def test_nested_duplicate_faces_are_merged_but_separate_objects_remain_ambiguous():
    full=box_mask();top=np.zeros_like(full);top[219:262,299:342]=1
    mapped,summary=mapped_instances(results([full,top]),'cube',full.shape)
    assert summary['raw_masks']==2 and summary['unique_masks']==1
    np.testing.assert_array_equal(mapped[0].masks.data[0],full.astype(bool))
    separate=box_mask((390,240))
    mapped,summary=mapped_instances(results([full,separate]),'cube',full.shape)
    assert summary['unique_masks']==2


def test_roi_mapping_never_accepts_cropped_masks_or_low_confidence():
    roi=(160,80,480,400);mask=np.zeros((320,320));mask[134:187,134:187]=1
    mapped,summary=mapped_instances(results([mask]),'cube',(480,640),roi)
    y,x=np.nonzero(mapped[0].masks.data[0]);assert x.mean()==320 and y.mean()==240
    mask[:,0:20]=1
    _,summary=mapped_instances(results([mask]),'cube',(480,640),roi)
    assert summary['unique_masks']==1 and summary['crop_clipped_components']==1
    _,summary=mapped_instances(results([mask],[.204]),'cube',(480,640),roi)
    assert summary['raw_masks']==0 and summary['unique_masks']==0


def test_roi_bounds_include_calibrated_target_and_motion_envelope():
    c=calibration();pose=np.eye(4);pose[2,3]=.4
    roi=search_roi(obb(),pose@np.asarray(c['T_tcp_camera']),c,.02)
    assert roi and roi[0]<290<350<roi[2] and roi[1]<210<270<roi[3]
    assert roi[2]-roi[0]>=320 and roi[3]-roi[1]>=320


def test_missing_full_frame_target_recovers_from_roi_without_lowering_confidence(tmp_path):
    c=calibration();pose=np.eye(4);pose[2,3]=.4
    class Detector:
        def __init__(self):self.calls=[]
        def predict(self,image,**kwargs):
            self.calls.append((image.shape,kwargs))
            assert kwargs['conf']==.25 and kwargs['retina_masks'] is True
            if image.shape[:2]==(480,640):return results([])
            mask=np.zeros(image.shape[:2]);h,w=mask.shape;mask[h//2-26:h//2+27,w//2-26:w//2+27]=1
            return results([mask])
    detector=Detector();diagnostics={}
    shift,mask,_=detect_translation(detector,np.zeros((480,640,3),np.uint8),obb(),pose,c,'cube',.02,diagnostics,tmp_path)
    np.testing.assert_allclose(shift,0,atol=1e-8)
    assert mask.shape==(480,640) and len(detector.calls)==2
    assert len(diagnostics['attempts'])==2 and diagnostics['view']=='roi'
    assert diagnostics['status']=='passed' and 'error' not in diagnostics
    assert (tmp_path/'wrist_roi.jpg').is_file()


def test_ambiguous_full_frame_is_not_resolved_by_discarding_context(tmp_path):
    c=calibration();pose=np.eye(4);pose[2,3]=.4;calls=[]
    def predict(image,**kwargs):
        calls.append(1);return results([box_mask((300,240)),box_mask((340,240))])
    with pytest.raises(WristTargetError) as error:
        detect_translation(SimpleNamespace(predict=predict),np.zeros((480,640,3),np.uint8),obb(),pose,c,'cube',.02,{},tmp_path)
    assert error.value.count==2 and len(calls)==1


def test_roi_surface_and_isolated_noise_are_screened_as_separate_hypotheses(tmp_path):
    c=calibration();pose=np.eye(4);pose[2,3]=.4
    def predict(image,**kwargs):
        if image.shape[:2]==(480,640):return results([])
        h,w=image.shape[:2];full=np.zeros((h,w));full[h//2-26:h//2+27,w//2-26:w//2+27]=1
        full[10:24,10:30]=1  # Isolated noise that used to reject the entire instance mask.
        top=np.zeros((h,w));top[h//2-21:h//2+22,w//2-21:w//2+22]=1
        return results([full,top],[.63,.56])
    diagnostics={}
    shift,mask,_=detect_translation(SimpleNamespace(predict=predict),np.zeros((480,640,3),np.uint8),obb(),pose,c,'cube',.02,diagnostics,tmp_path)
    assert mask.sum()==53*53 and np.linalg.norm(shift)<1e-8
    assert diagnostics['raw_masks']==2 and diagnostics['unique_masks']==2
    assert sum(v['plausible'] for v in diagnostics['detections'])==1


def test_two_plausible_components_in_one_mask_are_still_ambiguous(tmp_path):
    c=calibration();pose=np.eye(4);pose[2,3]=.4
    # Two disconnected 40x40 surfaces each fit the size/offset bounds. Do not pick one.
    mask=np.zeros((480,640));mask[220:260,278:318]=1;mask[220:260,322:362]=1
    def predict(image,**kwargs):return results([mask])
    with pytest.raises(WristTargetError) as error:
        detect_translation(SimpleNamespace(predict=predict),np.zeros((480,640,3),np.uint8),obb(),pose,c,'cube',.02,{},tmp_path)
    assert error.value.count==2


def test_partial_face_cannot_replace_a_consistent_tracked_silhouette():
    from hei_agent_bridge.grasp.wrist import estimate_translation
    c=calibration();pose=np.eye(4);pose[2,3]=.4
    _,_,first=estimate_translation(obb(),pose,c,results([box_mask()]),'cube',.02)
    ratio=first['selected_area_fraction']
    partial=np.zeros((480,640));partial[225:265,294:347]=1
    diagnostics={}
    with pytest.raises(WristTargetError):
        estimate_translation(obb(),pose,c,results([partial]),'cube',.02,diagnostics,reference_area_fraction=ratio)
    assert diagnostics['detections'][0]['silhouette_consistent'] is False
    # A fresh complete observation still passes the same 2 cm geometric and .25 detector gates.
    _,_,good=estimate_translation(obb(),pose,c,results([box_mask((322,240))]),'cube',.02,
                                 reference_area_fraction=ratio)
    assert good['detections'][0]['silhouette_consistent'] is True
