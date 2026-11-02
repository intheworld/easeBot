"""Observed rectangular container geometry, independent of placement coverage.

Four measured side profiles define the inner envelope. A separate metric grid
records observed/inferred support, occupied and unknown cells. Missing returns
never resize the container; bounded interpolation is explicitly distinguished.
"""
import cv2
import numpy as np

UNKNOWN, SUPPORT, OCCUPIED = 0, 1, 2
# User-approved tolerance for mildly tilted tray bottoms.
MAX_SUPPORT_TILT_DEG = 5.0


def container_component(world, observation):
    """Keep a dominant connected container instance, not detached mask speckles.

    This only selects boundary/support fitting pixels. The full depth image is
    retained for occupancy, so removed segmentation is never declared free.
    """
    mask=np.asarray(observation['mask'],bool)
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
    if count<=1:raise ValueError('Empty container mask')
    areas=stats[1:,cv2.CC_STAT_AREA];label=1+int(np.argmax(areas))
    report=dict(component_count=count-1,selected_pixels=int(areas.max()),
                excluded_pixels=int(areas.sum()-areas.max()))
    if count==2:return world,observation,report
    if areas.max()<.9*areas.sum():
        raise ValueError('Container mask has multiple substantial disconnected components')
    selected=labels==label
    transform=np.asarray(observation['camera_to_footprint'],float)
    k=np.asarray(observation['intrinsic'],float).reshape(3,3)
    camera=(world-transform[:3,3])@transform[:3,:3]
    valid=np.isfinite(camera).all(axis=1)&(camera[:,2]>0)
    uv=np.zeros((len(world),2),int)
    uv[valid]=np.rint(camera[valid,:2]/camera[valid,2,None]*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]).astype(int)
    valid&=(uv[:,0]>=0)&(uv[:,0]<mask.shape[1])&(uv[:,1]>=0)&(uv[:,1]<mask.shape[0])
    indices=np.flatnonzero(valid)
    keep=indices[selected[uv[indices,1],uv[indices,0]]]
    if len(keep)<150:raise ValueError('Dominant container component has too few depth points')
    return world[keep],{**observation,'mask':selected},report


def measured_floor(world, diagnostics=None):
    z=world[:,2];bins=np.floor(z/.006).astype(int)
    values,counts=np.unique(bins,return_counts=True);planes=[]
    for value,count in sorted(zip(values,counts),key=lambda pair:pair[1],reverse=True):
        height=float(np.median(z[bins==value]));near=np.abs(z-height)<.006
        if near.sum()<max(80,len(z)*.08) or any(abs(height-p)<.012 for p in planes):continue
        planes.append(float(np.median(z[near])))
    if not planes:raise ValueError('Container bottom not sufficiently observed')
    floor=min(planes);mask=np.abs(z-floor)<.006;points=world[mask]
    _,_,axes=np.linalg.svd(points-np.median(points,axis=0),full_matrices=False)
    normal_z=float(abs(axes[-1,2]));std=float(np.std(points[:,2]))
    report=dict(height_m=floor,point_count=len(points),normal_abs_z=normal_z,
                tilt_deg=float(np.degrees(np.arccos(np.clip(normal_z,0.,1.)))),
                height_std_m=std,max_tilt_deg=MAX_SUPPORT_TILT_DEG,
                max_height_std_m=.0035,tilt_exceeded=bool(normal_z<np.cos(np.deg2rad(MAX_SUPPORT_TILT_DEG))),noise_exceeded=std>.0035)
    if diagnostics is not None:diagnostics['support_plane']=report
    if report['tilt_exceeded'] or report['noise_exceeded']:
        raise ValueError('Support plane rejected: '
                         f"tilt={report['tilt_deg']:.3f} deg (max {report['max_tilt_deg']:.3f}), "
                         f"height_std={std*1000:.3f} mm (max 3.500); inspect region_diagnostics.json")
    return floor,mask


def side_profile(projected,z,floor,low,high,axis,sign):
    other=1-axis;middle=(low+high)/2;span=high[other]-low[other]
    use=(z>floor+.015)&(sign*(projected[:,axis]-middle[axis])>0)
    use&=(projected[:,other]>low[other]+.25*span)&(projected[:,other]<high[other]-.25*span)
    points=projected[use];heights=z[use]-floor
    if len(points)<10:return None
    values=points[:,axis];bins,counts=np.unique(np.floor(values/.004),return_counts=True)
    peak=(bins[np.argmax(counts)]+.5)*.004
    near=abs(values-peak)<.012;values=values[near];heights=heights[near];points=points[near]
    if len(values)<10 or np.ptp(points[:,other])<.3*span:
        return None
    # A vertical side is adequate when only a narrow upper rim is visible.
    # With measured height span, fit taper and include residual uncertainty in
    # the inward envelope, rather than taking the outermost silhouette as free.
    if np.ptp(heights)>.012:
        design=np.c_[heights,np.ones(len(values))]
        slope,offset=np.linalg.lstsq(design,values,rcond=None)[0]
    else:slope=0.;offset=float(np.median(values))
    residual=values-(slope*heights+offset)
    uncertainty=float(np.quantile(abs(residual),.95)+.002)
    if uncertainty>.014 or abs(slope)>1.:
        raise ValueError('Container side profile is ambiguous or not rectangular')
    return dict(axis=axis,sign=sign,slope=float(slope),offset=float(offset),
                uncertainty_m=uncertainty,point_count=len(values),
                observed_height_range_m=[float(heights.min()),float(heights.max())])


def grid_cells(xy,low,step,shape):
    cells=np.floor((xy-low)/step).astype(int)
    valid=np.all((cells>=0)&(cells<shape),axis=1)
    return cells,valid


def estimate_container(world,*,depth_observation=None,diagnostics=None):
    from ..arm.regions import region_geometry
    world=np.asarray(world,float)
    component_report=None
    if depth_observation is not None:
        world,depth_observation,component_report=container_component(world,depth_observation)
        if diagnostics is not None:diagnostics['container_component']=component_report
    floor,floor_mask=measured_floor(world,diagnostics);z=world[:,2]
    rim=z>floor+.015
    if rim.sum()<40:raise ValueError('Container bottom/rim separation not observed')
    height=float(np.quantile(z[rim],.98)-floor)
    if not .015<=height<=.5:raise ValueError('Unsupported container height')
    # Trim isolated rim speckles before estimating axes. Bottom depth holes do not
    # participate in this orientation or in the four side estimates.
    fp=world[rim,:2];lo,hi=np.quantile(fp,[.01,.99],axis=0)
    retained=fp[np.all((fp>=lo)&(fp<=hi),axis=1)]
    corners=cv2.boxPoints(cv2.minAreaRect(retained.astype(np.float32))).astype(float)
    axis=corners[1]-corners[0];axis/=np.linalg.norm(axis)
    # Canonical yaw avoids 90-degree flips between equivalent rectangles.
    yaw=(np.arctan2(axis[1],axis[0])+np.pi/4)%(np.pi/2)-np.pi/4
    rotation=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    xy=world[:,:2]@rotation;low,high=np.quantile(xy[rim],[.02,.98],axis=0)
    sides=[]
    floor_low,floor_high=np.quantile(xy[floor_mask],[.02,.98],axis=0)
    for a in (0,1):
        for sign in (-1,1):
            side=side_profile(xy,z,floor,low,high,a,sign)
            if side is None:
                # Crop to observed support; do not infer an unseen container edge.
                side=dict(axis=a,sign=sign,slope=0.,
                          offset=float((floor_low if sign<0 else floor_high)[a]),
                          uncertainty_m=.005,source='observed_support_extent',
                          boundary_observed=False)
            else:
                side.update(source='observed_side_profile',boundary_observed=True)
            sides.append(side)
    inner_low=np.zeros(2);inner_high=np.zeros(2)
    for side in sides:
        ends=side['offset']+side['slope']*np.array([0.,height])
        if side['sign']<0:inner_low[side['axis']]=max(ends)+side['uncertainty_m']
        else:inner_high[side['axis']]=min(ends)-side['uncertainty_m']
    size=inner_high-inner_low
    if min(size)<.03 or max(size)>2:raise ValueError('Invalid measured container inner envelope')
    center=(inner_low+inner_high)/2;position=rotation@center
    region=region_geometry(np.eye(4),*position,floor,*size,float(yaw),'inside',height,.005)
    region['schema']='hei.place_region/v2'
    outer_low=np.minimum(np.quantile(xy,.005,axis=0)-.005,inner_low-.005)
    outer_high=np.maximum(np.quantile(xy,.995,axis=0)+.005,inner_high+.005)
    region['outer_bounds_xy_m']=[*(outer_low-center).tolist(),*(outer_high-center).tolist()]
    step=.004;shape=np.ceil(size/step).astype(int)
    if max(shape)>250:raise ValueError('Container grid too large')
    grid=np.zeros(tuple(shape[::-1]),np.uint8)
    # Use full aligned RGB-D within the container projection, including objects
    # the target segmentation excluded. Valid elevated/deeper returns win over
    # support; dropouts and unobserved cells remain explicitly unknown.
    observed=world
    if depth_observation is not None:
        depth=np.asarray(depth_observation['depth'],float)
        k=np.asarray(depth_observation['intrinsic'],float).reshape(3,3)
        transform=np.asarray(depth_observation['camera_to_footprint'],float)
        if depth.ndim!=2 or not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0:
            raise ValueError('Invalid container depth observation')
        from .transforms import validate_transforms
        transform=validate_transforms(transform[None],'camera_to_footprint')[0]
        v,u=np.nonzero(np.isfinite(depth)&(depth>0));d=depth[v,u]
        camera=np.c_[(u-k[0,2])*d/k[0,0],(v-k[1,2])*d/k[1,1],d]
        observed=camera@transform[:3,:3].T+transform[:3,3]
    observed_xy=observed[:,:2]@rotation
    oc,inside=grid_cells(observed_xy,inner_low,step,shape)
    inside&=np.all((observed_xy>=inner_low)&(observed_xy<inner_high),axis=1)
    recovery=None;recovered=np.empty((0,3));surface_samples=np.zeros(len(observed),bool)
    if depth_observation is not None:
        from .support_surface import recover_surface
        recovered,recovery,surface_mask=recover_surface(depth,depth_observation['mask'],k,transform,floor)
        surface_samples=surface_mask[v,u]
    # Bottom classification must not depend on whether the camera happens to
    # produce a missing-depth hole beside a measured point.
    from .support_surface import measured_surface
    training=inside.copy()
    if depth_observation is not None:
        training &= np.asarray(depth_observation['mask'],bool)[v,u]
    measured_support,surface_report=measured_surface(observed,training,floor,max_tilt_deg=MAX_SUPPORT_TILT_DEG)
    surface_samples |= measured_support
    # Compare coherent curved patches to their fitted surface. Preserve actual
    # measured heights for a conservative release/support elevation below.
    elevated=inside&(observed[:,2]>floor+.006)&~surface_samples
    grid[:]=UNKNOWN;support_upper=floor
    occupied_tops=np.full(grid.shape,floor,float)
    for iy,ix in np.ndindex(grid.shape):
        selected=inside&(oc[:,0]==ix)&(oc[:,1]==iy)
        heights=observed[selected,2]
        if not len(heights):continue
        residual=heights-floor
        residual[surface_samples[selected]]=0.
        lower,upper=np.quantile(residual,[.05,.95])
        if upper>.006:
            grid[iy,ix]=OCCUPIED
            occupied_tops[iy,ix]=float(np.max(heights))
        elif lower>=-.006:
            grid[iy,ix]=SUPPORT
            support_upper=max(support_upper,float(np.quantile(heights,.95)))
    rc,rv=grid_cells(recovered[:,:2]@rotation,inner_low,step,shape)
    rv&=np.all((recovered[:,:2]@rotation>=inner_low)&(recovered[:,:2]@rotation<inner_high),axis=1)
    for point,(ix,iy) in zip(recovered[rv],rc[rv]):
        if not np.any(inside&(oc[:,0]==ix)&(oc[:,1]==iy)):
            grid[iy,ix]=3;support_upper=max(support_upper,float(point[2]))
    # The release reference and collision floor enclose the observed curved
    # bottom from above; no obstacle is removed while keeping a lower goal.
    region['T_footprint_region'][2][3]=support_upper
    # Preserve absolute measured rim height when moving the support origin.
    wall_above_support=max(.001,floor+height-support_upper)
    region['wall_height_m']=wall_above_support
    # Grid cells straddling the inner envelope are unknown, never enlarged by ceil.
    if shape[0]*step>size[0]+1e-9:grid[:,-1]=UNKNOWN
    if shape[1]*step>size[1]+1e-9:grid[-1,:]=UNKNOWN
    occupied_height=max(height,float(np.max(observed[elevated,2])-floor) if elevated.any() else height)
    widths=np.minimum(step,np.maximum(0.,size[0]-np.arange(shape[0])*step))
    lengths=np.minimum(step,np.maximum(0.,size[1]-np.arange(shape[1])*step))
    cell_area=lengths[:,None]*widths[None,:]
    area=lambda label:float(cell_area[grid==label].sum())
    model=dict(schema='hei.container_space/v2',boundary=dict(source=('four_observed_side_profiles' if all(s['boundary_observed'] for s in sides)
                                            else 'partial_sides_with_observed_support_crop'),
               inner_size_xy_m=size.tolist(),sides=sides),
               support=dict(height_m=support_upper,measured_reference_height_m=floor,point_count=int(floor_mask.sum()),
                            height_std_m=float(np.std(z[floor_mask]))),
               coverage=dict(origin_xy_m=(-size/2).tolist(),resolution_m=step,shape_xy=shape.tolist(),
                             labels={'unknown':UNKNOWN,'observed_support':SUPPORT,'observed_occupied':OCCUPIED,'inferred_support':3},
                             cells=grid.tolist(),occupied_top_z_m=occupied_tops.tolist(),observed_support_area_m2=area(SUPPORT),
                             unknown_area_m2=area(UNKNOWN),occupied_area_m2=area(OCCUPIED),inferred_support_area_m2=area(3)),
               occupied_height_m=max(.001,floor+occupied_height-support_upper),unknown_height_m=wall_above_support,
               assumptions=['rectangular_container','horizontal_observed_bottom','linear_side_profile_envelope'],
               inference='bounded_local_quadratic_invalid_depth_only',depth_recovery=recovery,measured_surface=surface_report,
               container_component=component_report)
    region.update(geometry_scope='measured_container_with_separate_coverage',spatial_model=model,
                  support_point_count=int(floor_mask.sum()),support_height_std_m=float(np.std(z[floor_mask])),
                  grid_resolution_m=step)
    if diagnostics is not None:
        diagnostics['spatial_model']=model
        palette=np.array([[100,100,100],[40,180,40],[40,40,230],[220,180,40]],np.uint8)
        diagnostics['coverage_image']=cv2.resize(palette[grid],None,fx=12,fy=12,interpolation=cv2.INTER_NEAREST)
    return region
