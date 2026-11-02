"""Small measured supports retain metric extent without filling large unseen holes."""
from pathlib import Path
import numpy as np
import pytest
from hei_agent_bridge.grasp.place_region import estimate_region


def surface(width,length):
    x,y=np.meshgrid(np.arange(-width/2,width/2,.0008),np.arange(-length/2,length/2,.0008))
    return np.column_stack((x.ravel(),y.ravel(),np.full(x.size,.4)))


def test_saved_red_block_passes_surface_model_without_expanding_measured_extent():
    points=np.load(Path(__file__).parent/'fixtures/on_support_d286.npz')['points'];report={}
    region=estimate_region(points,np.eye(4),'on',diagnostics=report)
    assert region['grid_resolution_m']==.002
    assert min(region['size_xy_m'])>=.02-1e-10
    assert max(region['size_xy_m'])<.04
    assert report['support_grid']['boundary_inset_m']==.002
    assert all(min(o['size_xy_m'])>.01 for o in region['usable_rectangle_options'])


def test_three_centimeter_support_keeps_observed_top():
    report={};region=estimate_region(surface(.032,.038),np.eye(4),'on',diagnostics=report)
    assert min(region['size_xy_m'])>=.026
    assert max(region['size_xy_m'])<=.038
    assert region['grid_resolution_m']==.002


def test_large_support_retains_existing_grid_and_inset():
    region=estimate_region(surface(.25,.30),np.eye(4),'on')
    assert region['grid_resolution_m']==.008


def test_small_surface_does_not_bridge_unobserved_split():
    points=surface(.07,.06);points=points[np.abs(points[:,0])>.006]
    region=estimate_region(points,np.eye(4),'on')
    # No returned rectangle may straddle the 12 mm unseen stripe at world X=0.
    for option in [region,*region['usable_rectangle_options']]:
        t=np.array(option['T_footprint_region']);size=np.array(option['size_xy_m'])
        corners=np.array([[-1,-1],[-1,1],[1,-1],[1,1]])*size/2
        xx=(corners@t[:2,:2].T+t[:2,3])[:,0]
        assert xx.min()>=.006-1e-9 or xx.max()<=-.006+1e-9


def test_on_overhang_candidates_keep_estimated_com_inside_support(tmp_path):
    from test_arm_transport import make_arm_service,attach,URDF
    from test_place_regions import declared,plan_args
    from hei_agent_bridge.arm.regions import region_geometry
    from hei_agent_bridge.arm.place import region_place_candidates,placement_targets
    from hei_agent_bridge.arm.geometry import base_from_frame
    service,_,backend=make_arm_service(tmp_path);obj=attach(service,backend)
    args=declared(service,backend,obj,'on');args.update(size_x_m=.022,size_y_m=.020)
    region=region_geometry(np.eye(4),*[args[k] for k in ('x_m','y_m','support_z_m','size_x_m',
        'size_y_m','yaw_rad','relation','wall_height_m','boundary_thickness_m')]);region['id']='small-top'
    request=dict(urdf=str(URDF),profile=service.profile,state=backend.state,carry=service.carry.snapshot(),
        arguments={**plan_args('small-top'),'region':region,'path_mode':'linear'})
    frame=base_from_frame(URDF,backend.state,'base_footprint');mount=np.array(request['carry']['payload']['T_tcp_object'])
    candidates=list(region_place_candidates(request));assert candidates
    for candidate in candidates:
        goal=placement_targets(candidate)[3]
        com=np.linalg.inv(np.array(region['T_footprint_region']))@np.linalg.inv(frame)@goal@mount
        assert (np.abs(com[:2,3])<=np.array(region['size_xy_m'])/2-.008+1e-9).all()
    invalid={**candidates[0],'arguments':{**candidates[0]['arguments'],'x_m':args['x_m']+.10}}
    with pytest.raises(ValueError,match='COM projection'):placement_targets(invalid)
    container={**region,'relation':'inside','wall_height_m':.03}
    invalid={**request,'arguments':{**request['arguments'],'region':container}}
    with pytest.raises(ValueError,match='does not fit'):list(region_place_candidates(invalid))
