"""Partial depth coverage policy retains measured collisions and exact strict mode."""
from copy import deepcopy
import numpy as np
import pytest
from hei_agent_bridge.arm.spatial import supported_centers,exclusion_boxes,unknown_fraction


def region(cells):
    grid=np.asarray(cells)
    return {'size_xy_m':[grid.shape[1]*.01,grid.shape[0]*.01],
        'T_footprint_region':np.eye(4).tolist(),
        'spatial_model':{'schema':'hei.container_space/v2','unknown_height_m':.05,'occupied_height_m':.05,
            'coverage':{'cells':grid.tolist(),'origin_xy_m':[-grid.shape[1]*.005,-grid.shape[0]*.005],
                        'resolution_m':.01,'shape_xy':list(grid.shape[::-1])}}}


def centers(r):
    # Footprint covers four complete cells at the only candidate.
    poly=np.array([[-.01,-.01],[.01,-.01],[.01,.01],[-.01,.01]])
    return list(supported_centers(r,np.array([.01,.01]),np.zeros(2),poly))


def test_half_unknown_allowed_but_observed_occupied_always_blocks():
    r=region([[0,1],[0,1]])
    assert not centers(r)
    r['max_unknown_support_fraction']=.5
    assert len(centers(r))==1
    assert not exclusion_boxes(r)  # Missing depth does not become a solid wall.
    r['spatial_model']['coverage']['cells'][1][0]=2
    assert not centers(r)
    assert {b['kind'] for b in exclusion_boxes(r)}=={'occupied'}


def test_over_allowance_all_unknown_and_strict_collision_remain_rejected():
    r=region([[0,0],[0,1]]);r['max_unknown_support_fraction']=.5
    assert not centers(r)
    r['spatial_model']['coverage']['cells']=[[0,0],[0,0]]
    assert not centers(r)
    r['max_unknown_support_fraction']=0
    assert {b['kind'] for b in exclusion_boxes(r)}=={'unknown'}


@pytest.mark.parametrize('value',[-.1,1.1,True,None,'0.5',float('nan')])
def test_policy_rejects_invalid_configuration(value):
    with pytest.raises(ValueError,match='max_unknown'):unknown_fraction({'max_unknown_support_fraction':value})


def test_partial_cells_are_counted_conservatively():
    r=region([[0,1],[0,1]]);r['max_unknown_support_fraction']=.49
    assert not centers(r)
    r['max_unknown_support_fraction']=.5
    assert centers(r)


def test_saved_failure_gets_candidates_without_clearing_observed_obstacles():
    import json
    from pathlib import Path
    from hei_agent_bridge.arm.place import region_place_candidates
    req=json.loads((Path(__file__).parent/'fixtures/unknown_support_request.json').read_text())
    req['urdf']=str(Path(__file__).resolve().parents[1]/'src/hei_description/urdf/hei_body.urdf')
    with pytest.raises(ValueError,match='no support footprint'):
        list(region_place_candidates(req))
    r=req['arguments']['region'];before=deepcopy(r['spatial_model']['coverage']['cells'])
    r['max_unknown_support_fraction']=.5
    candidates=list(region_place_candidates(req))
    assert candidates
    assert r['spatial_model']['coverage']['cells']==before
    assert all(b['kind']=='occupied' for b in exclusion_boxes(r))


def test_vectorized_strict_mode_matches_original_sat_decisions():
    rng=np.random.default_rng(14)
    for _ in range(8):
        grid=rng.choice([0,1,2,3],(8,8),p=[.05,.8,.05,.1]);r=region(grid)
        extent=np.array([.008,.011]);available=np.array([.025,.02])
        polygon=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*extent
        step=.01;origin=np.array([-.04,-.04]);by,bx=np.nonzero(~np.isin(grid,[1,3]))
        blocked=origin+(np.c_[bx,by]+.5)*step
        counts=2*np.ceil(available/(step/8)).astype(int)+1
        xx,yy=np.meshgrid(np.linspace(-available[0],available[0],counts[0]),np.linspace(-available[1],available[1],counts[1]))
        samples=np.c_[xx.ravel(),yy.ravel()];samples=samples[np.argsort((samples*samples).sum(1),kind='stable')]
        expected=[]
        for c in samples:
            separate=(blocked+step/2<=c-extent+1e-10)|(blocked-step/2>=c+extent-1e-10)
            if np.all(np.any(separate,axis=1)):expected.append(c)
        actual=list(supported_centers(r,extent,available,polygon))
        np.testing.assert_allclose(np.array(actual).reshape(-1,2),np.array(expected).reshape(-1,2),atol=1e-15)
