"""Explicitly permitted inclination search; no ROS or robot motion."""
from pathlib import Path
from itertools import product
import json
import cv2
import numpy as np
import pytest
from hei_agent_bridge.arm.place import region_place_candidates,placement_targets,placement_rotation
from hei_agent_bridge.arm.geometry import base_from_frame
from hei_agent_bridge.grasp.planner import RightArmPlanner
from test_arm_transport import make_arm_service,URDF


def saved_request():
    r=json.loads((Path(__file__).parent/'fixtures/place_tilt_1510.json').read_text());r['urdf']=str(URDF)
    return r


def test_explicit_zero_preserves_inclination():
    r=saved_request();r['arguments']['max_tilt_deg']=0.;cs=list(region_place_candidates(r));assert len(cs)==32
    original=RightArmPlanner(URDF).fk(r['state']['right'])[:3,:3]
    for c in cs:
        assert '_placement_tilt_rad' not in c['arguments']
        rotation=placement_targets(c)[3][:3,:3]
        up=base_from_frame(URDF,r['state'],'base_footprint')[:3,2]
        np.testing.assert_allclose(up@rotation,up@original,atol=1e-12)


def test_saved_unreachable_pose_gains_valid_ik_with_explicit_five_degree_tilt():
    r=saved_request();r['arguments']['max_tilt_deg']=5.;cs=list(region_place_candidates(r));p=RightArmPlanner(URDF)
    assert len(cs)==64 and all('_placement_tilt_rad' not in c['arguments'] for c in cs[:32])
    accepted=[]
    for c in cs[32:]:
        goal=placement_targets(c)[3]
        try:q=p.solve(goal,np.array(r['state']['right']))
        except ValueError:continue
        actual=p.fk(q)
        assert np.linalg.norm(actual[:3,3]-goal[:3,3])<=.005
        assert np.linalg.norm(cv2.Rodrigues(goal[:3,:3].T@actual[:3,:3])[0])<=np.deg2rad(3)
        accepted.append(c);break
    assert accepted
    c=accepted[0];goal=placement_targets(c)[3]
    frame=base_from_frame(URDF,r['state'],'base_footprint');mount=np.array(r['carry']['payload']['T_tcp_object'])
    obj=np.linalg.inv(frame)@goal@mount
    size=np.array(r['carry']['payload']['size_m']);corners=np.array(list(product([-1.,1.],repeat=3)))*size/2
    xyz=corners@obj[:3,:3].T+obj[:3,3]
    # Height uses the TILTED box, not its old upright half-height.
    clearance=placement_targets(c)[4]
    assert xyz[:,2].min()==pytest.approx(c['arguments']['support_z_m']+clearance,abs=1e-10)
    np.testing.assert_allclose(obj[:2,3],[c['arguments']['x_m'],c['arguments']['y_m']],atol=1e-12)


@pytest.mark.parametrize('value',[-1,11,float('nan'),True])
def test_tilt_permission_is_bounded_and_explicit(value):
    r=saved_request();r['arguments']['max_tilt_deg']=value
    with pytest.raises(ValueError,match='max_tilt'):list(region_place_candidates(r))


def test_internal_tilt_cannot_exceed_caller_permission():
    r=saved_request();r['arguments']['max_tilt_deg']=0.;r['arguments']['_placement_tilt_rad']=np.deg2rad(5)
    with pytest.raises(ValueError,match='explicitly permitted'):placement_rotation(r)


def test_discovered_tilt_skill_requires_a_separate_explicit_contract(tmp_path):
    service,skills,backend=make_arm_service(tmp_path)
    old=skills._definitions['plan_object_placement'];new=skills._definitions['plan_object_placement_with_tilt']
    assert 'max_tilt_deg' not in old['arguments']
    assert new['arguments']['max_tilt_deg']=={'type':'number','minimum':0,'maximum':10}
    assert not new['motion']


def test_default_allows_ten_degree_search_after_original_candidates():
    r=saved_request();cs=list(region_place_candidates(r))
    assert len(cs)==96
    assert all('_placement_tilt_rad' not in c['arguments'] for c in cs[:32])
    assert {round(float(np.rad2deg(c['arguments'].get('_placement_tilt_rad',0))),6) for c in cs}=={0.,5.,10.}


def test_catalog_advertises_authorized_placement_defaults(tmp_path):
    _,skills,_=make_arm_service(tmp_path)
    for name in ('plan_object_placement','plan_object_placement_with_tilt'):
        description=skills._definitions[name]['description']
        for key in ('approach_height_m','retreat_height_m','release_gap_m'):
            assert key+'=0.05' in description
        assert 'user-specified values override' in description
