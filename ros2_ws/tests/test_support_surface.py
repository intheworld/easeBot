"""Curved support inference must retain obstacles and unbounded unknown space."""
import json
from pathlib import Path
import cv2
import numpy as np
import pytest
from hei_agent_bridge.grasp.support_surface import recover_surface
from hei_agent_bridge.arm.spatial import supported_centers


def scene():
    v,u=np.indices((100,120));depth=.6-.02*((u-60)/50)**2
    mask=np.zeros(depth.shape,bool);mask[8:92,8:112]=True
    transform=np.diag([1.,-1.,-1.,1.]);transform[2,3]=1.
    return depth,mask,[300,0,60,0,300,50,0,0,1],transform


def test_curved_hole_keeps_measured_values_and_reports_inference():
    depth,mask,k,t=scene();depth[44:56,54:66]=np.nan;before=depth.copy()
    points,report,support=recover_surface(depth,mask,k,t,.4)
    assert len(points)>100 and report['inferred_support_pixels']==len(points)
    assert np.array_equal(depth,before,equal_nan=True)
    assert np.isfinite(depth[support]).all()
    assert .398<points[:,2].min()<.403


@pytest.mark.parametrize('kind',['edge','large','valid_background','valid_obstacle','noisy_ring'])
def test_unbounded_or_inconsistent_surface_not_filled(kind):
    depth,mask,k,t=scene()
    if kind=='edge':depth[:20,50:65]=np.nan
    elif kind=='large':depth[20:80,30:90]=np.nan
    elif kind=='valid_background':depth[44:56,54:66]=.9
    elif kind=='valid_obstacle':depth[44:56,54:66]=.5
    else:
        depth[44:56,54:66]=np.nan
        ring=np.zeros(depth.shape,np.uint8);ring[44:56,54:66]=1
        ring=cv2.dilate(ring,np.ones((13,13),np.uint8)).astype(bool)&np.isfinite(depth)
        rng=np.random.default_rng(17);depth[ring]+=rng.uniform(-.03,.03,ring.sum())
    points,_,_=recover_surface(depth,mask,k,t,.4)
    assert not len(points)


def test_saved_failed_footprint_has_supported_candidate_with_exact_projection():
    data=json.loads((Path(__file__).parent/'fixtures/container_959_footprint.json').read_text())
    region=data['region'];extent=np.asarray(data['extent']);available=np.asarray(data['available'])
    candidates=list(supported_centers(region,extent,available,np.asarray(data['polygon'])))
    assert candidates
    assert not list(supported_centers(region,extent,available))
    # An actual obstacle in the otherwise feasible footprint still blocks it.
    region['spatial_model']['coverage']['cells']=np.full_like(region['spatial_model']['coverage']['cells'],2).tolist()
    assert not list(supported_centers(region,extent,available,np.asarray(data['polygon'])))


def test_measured_curved_bottom_without_depth_holes_preserves_obstacles():
    from hei_agent_bridge.grasp.support_surface import measured_surface
    x,y=np.meshgrid(np.linspace(-.06,.06,81),np.linspace(-.06,.06,81))
    z=.4+.01*(1-(x/.06)**2)
    obstacle=(abs(x)<.012)&(abs(y)<.012)
    z[obstacle]+=.025
    points=np.c_[x.ravel(),y.ravel(),z.ravel()];before=points.copy()
    support,report=measured_surface(points,np.ones(len(points),bool),.4)
    assert report['accepted']
    assert support[~obstacle.ravel()].mean()>.98
    assert not support[obstacle.ravel()].any()
    np.testing.assert_array_equal(points,before)


def test_measured_bottom_keeps_valid_deeper_returns_and_outside_hull():
    from hei_agent_bridge.grasp.support_surface import measured_surface
    x,y=np.meshgrid(np.linspace(-.06,.06,41),np.linspace(-.06,.06,41))
    points=np.c_[x.ravel(),y.ravel(),(.4+x*np.tan(np.deg2rad(4))).ravel()]
    points=np.vstack((points,[0,0,.37],[.3,0,.4],[0,0,.43]))
    training=np.ones(len(points),bool);training[-3:]=False
    support,report=measured_surface(points,training,.4)
    assert report['accepted']
    assert support[:-3].mean()>.98
    assert not support[-3:].any()


def test_measured_surface_rejects_excessive_tilt():
    from hei_agent_bridge.grasp.support_surface import measured_surface
    x,y=np.meshgrid(np.linspace(-.01,.01,41),np.linspace(-.01,.01,41))
    points=np.c_[x.ravel(),y.ravel(),(.4+x*.3).ravel()]
    support,report=measured_surface(points,np.ones(len(points),bool),.4)
    assert not report['accepted'] and not support.any()


def test_saved_empty_tray_valid_depth_is_classified_without_holes():
    from hei_agent_bridge.grasp.support_surface import measured_surface
    with np.load(Path(__file__).parent/'fixtures/container_measured_bottom.npz') as data:
        points=data['points'];floor=float(data['floor'])
    support,report=measured_surface(points,np.ones(len(points),bool),floor)
    assert report['accepted']
    assert support.mean()>.85
    assert np.count_nonzero(support&(points[:,2]>floor+.006))>500
    # Rim/high returns are not absorbed into the fitted bottom.
    assert not support[points[:,2]>floor+.025].any()


def test_measured_surface_does_not_absorb_small_step():
    from hei_agent_bridge.grasp.support_surface import measured_surface
    x,y=np.meshgrid(np.linspace(-.06,.06,81),np.linspace(-.06,.06,81))
    z=np.full_like(x,.4);obstacle=(abs(x)<.025)&(abs(y)<.025);z[obstacle]+=.010
    points=np.c_[x.ravel(),y.ravel(),z.ravel()]
    support,report=measured_surface(points,np.ones(len(points),bool),.4)
    assert report['accepted'] and not support[obstacle.ravel()].any()


def test_occupied_boxes_use_absolute_measured_height():
    from hei_agent_bridge.arm.spatial import exclusion_boxes
    region={'size_xy_m':[.012,.004],'T_footprint_region':np.eye(4).tolist(),
        'spatial_model':{'schema':'hei.container_space/v2','unknown_height_m':.06,'occupied_height_m':.06,
        'coverage':{'cells':[[2,1,2]],'origin_xy_m':[-.006,-.002],'resolution_m':.004,'shape_xy':[3,1],
                    'occupied_top_z_m':[[.999,1.,1.008]]}}}
    region['T_footprint_region'][2][3]=1.
    boxes=exclusion_boxes(region)
    assert len(boxes)==1
    assert boxes[0]['size_m'][2]==pytest.approx(.008)
    assert boxes[0]['T_region_box'][2][3]==pytest.approx(.004)
    region['spatial_model']['coverage']['occupied_top_z_m']=[[float('nan')]*3]
    with pytest.raises(ValueError,match='height map'):exclusion_boxes(region)


def test_container_component_excludes_detached_mask_not_depth():
    from hei_agent_bridge.grasp.container_space import container_component
    mask=np.zeros((40,60),bool);mask[5:30,5:30]=True;mask[5:8,50:53]=True
    v,u=np.nonzero(mask);world=np.c_[u,v,np.ones(len(u))]
    depth=np.ones(mask.shape)
    obs=dict(mask=mask,depth=depth,intrinsic=np.eye(3),camera_to_footprint=np.eye(4))
    points,selected,report=container_component(world,obs)
    assert len(points)==625 and report['excluded_pixels']==9
    assert selected['depth'] is depth and obs['mask'][6,51]
    assert not selected['mask'][6,51] and selected['mask'][6,6]


def test_container_component_rejects_competing_fragments():
    from hei_agent_bridge.grasp.container_space import container_component
    import pytest
    mask=np.zeros((40,60),bool);mask[5:30,5:30]=True;mask[5:30,35:60]=True
    with pytest.raises(ValueError,match='substantial'):
        container_component(np.zeros((200,3)),dict(mask=mask))
