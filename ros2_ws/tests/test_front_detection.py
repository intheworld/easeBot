"""Explicit prompt recovery on synthetic saved RGB-D, no models or hardware."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/hei_agent_bridge/src'))
from hei_agent_bridge.grasp.detection import detect_cloud,prompt_aliases


class Detector:
    def __init__(self, responses): self.responses=responses;self.calls=[]
    def set_classes(self, names): self.prompt=names[0]
    def predict(self, image, **kwargs):
        self.calls.append((self.prompt,kwargs))
        return self.responses.get(self.prompt,[])


def result(name,mask=True,count=1):
    return [SimpleNamespace(names={0:name},boxes=SimpleNamespace(cls=np.zeros(count),conf=np.full(count,.4)),
        masks=SimpleNamespace(data=np.ones((count,10,10))) if mask else None)]


def detect(detector,tmp_path,aliases,depth=None):
    report={}
    output=detect_cloud(detector,np.zeros((10,10,3),np.uint8),
        np.ones((10,10)) if depth is None else depth,np.eye(3),'open box',np.eye(4),aliases,report,tmp_path)
    return output,report


def test_explicit_alias_recovers_without_changing_task_target(tmp_path):
    d=Detector({'gray container':result('gray container')})
    (score,points,mask,prompt),report=detect(d,tmp_path,{'open box':['gray container']})
    assert prompt=='gray container' and score==.4 and len(points)==100
    assert report['target']=='open box' and report['selected_prompt']==prompt
    assert [x['status'] for x in report['attempts']]==['rejected','selected']
    assert 'error' not in report
    assert (tmp_path/'perception_overlay.jpg').exists()
    assert all(call[1]['conf']==.25 and call[1]['retina_masks'] for call in d.calls)


def test_success_does_not_try_alias(tmp_path):
    d=Detector({'open box':result('open box')})
    _,report=detect(d,tmp_path,{'open box':['gray container']})
    assert len(d.calls)==1 and report['selected_prompt']=='open box'


@pytest.mark.parametrize('masks,depth',[(False,np.ones((10,10))),(True,np.full((10,10),np.nan))])
def test_no_fallback_after_segmentation_or_depth_failure(tmp_path,masks,depth):
    d=Detector({'open box':result('open box',masks),'container':result('container')})
    with pytest.raises(ValueError,match='No segmented RGB-D object'):
        detect(d,tmp_path,{'open box':['container']},depth)
    assert len(d.calls)==1


def test_ambiguity_survives_prompt_recovery(tmp_path):
    responses=result('container',count=2)
    responses[0].masks.data[0,5:,:]=0
    responses[0].masks.data[1,:5,:]=0
    d=Detector({'container':responses})
    _,report=detect(d,tmp_path,{'open box':['container']})
    assert report['segmented_detections']==2
    assert report['unique_segmented_objects']==2  # Distinct targets remain ambiguous.


@pytest.mark.parametrize('value',[[],{'box':'container'},{'box':['a','b','c','d']},{'box':['']},{'box\n':['a']}, {'box':['a'],' box':['b']}])
def test_invalid_alias_configuration(value):
    with pytest.raises(ValueError):prompt_aliases(value)


def test_no_automatic_class_expansion(tmp_path):
    d=Detector({'container':result('container')})
    with pytest.raises(ValueError):detect(d,tmp_path,{})
    assert [prompt for prompt,_ in d.calls]==['open box']


def test_duplicate_instance_masks_do_not_create_false_ambiguity(tmp_path):
    d=Detector({'open box':result('open box',count=2)})
    _,report=detect(d,tmp_path,{})
    assert report['segmented_detections']==2
    assert report['unique_segmented_objects']==1
    assert report['suppressed_duplicate_masks'][0]['mask_iou']==1.


def test_saved_red_block_duplicate_masks_keep_original_best_geometry(tmp_path):
    from hei_agent_bridge.grasp.obb import segmented_cloud
    masks=np.load(Path(__file__).parent/'fixtures/duplicate_red_masks_377a.npz')['masks']
    response=SimpleNamespace(names={0:'top of the red block'},
        boxes=SimpleNamespace(cls=np.zeros(2),conf=np.array([.910911,.573853])),
        masks=SimpleNamespace(data=masks))
    report={}
    score,_,selected=segmented_cloud([response],np.ones(masks.shape[1:]),np.eye(3),
        'top of the red block',np.eye(4),report)
    assert score==.910911 and np.array_equal(selected,masks[0])
    assert report['unique_segmented_objects']==1
    assert report['suppressed_duplicate_masks'][0]['mask_iou']==pytest.approx(.9226285)
