"""Conservative horizontal support regions from segmented metric RGB-D surfaces.

Horizontal surfaces retain observed support rectangles. Containers use separate
boundary and coverage layers in container_space; no hidden-bottom inference.
"""
import cv2
import numpy as np


def largest_rectangle(mask):
    """Largest all-true axis-aligned rectangle, returned as x0,y0,x1,y1 (exclusive)."""
    heights=np.zeros(mask.shape[1],int);best=(0,0,0,0);area=0
    for y,row in enumerate(mask):
        heights=np.where(row,heights+1,0);stack=[]
        for x,h in enumerate([*heights,0]):
            start=x
            while stack and stack[-1][1]>h:
                left,height=stack.pop();candidate=(x-left)*height
                if candidate>area:area=candidate;best=(left,y-height+1,x,y+1)
                start=left
            if h and (not stack or stack[-1][1]<h):stack.append((start,h))
    return best


def rectangle_candidates(mask):
    """Keep non-dominated width/height choices, not only maximum area.

    A long narrow maximum-area rectangle may not fit an object even though a
    slightly smaller, wider rectangle does. Every retained cell is supported.
    """
    heights=np.zeros(mask.shape[1],int);by_size={}
    for y,row in enumerate(mask):
        heights=np.where(row,heights+1,0);stack=[]
        for x,h in enumerate([*heights,0]):
            start=x
            while stack and stack[-1][1]>h:
                left,height=stack.pop();start=left
                if min(x-left,height)>=4:
                    by_size.setdefault((x-left,height),(left,y-height+1,x,y+1))
            if h and (not stack or stack[-1][1]<h):stack.append((start,h))
    keep=[];max_height=0
    for (width,height),rect in sorted(by_size.items(),reverse=True):
        if height>max_height:keep.append(rect);max_height=height
    return sorted(keep,key=lambda r:(r[2]-r[0])*(r[3]-r[1]),reverse=True)


def estimate_region(points,base_to_footprint,relation,*,depth_observation=None,diagnostics=None):
    from ..arm.regions import region_geometry
    if relation not in ('on','inside'):raise ValueError('Unsupported placement relation')
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<150 or not np.isfinite(points).all():
        raise ValueError('Placement needs at least 150 finite visible surface points')
    world=points@base_to_footprint[:3,:3].T+base_to_footprint[:3,3]
    if relation=='inside':
        from .container_space import estimate_container
        return estimate_container(world,depth_observation=depth_observation,diagnostics=diagnostics)
    # Find repeated horizontal surfaces; median within a 6 mm slab rejects vertical walls.
    z=world[:,2];bins=np.floor(z/.006).astype(int)
    values,counts=np.unique(bins,return_counts=True)
    peaks=sorted(zip(values,counts),key=lambda p:p[1],reverse=True)
    planes=[]
    for value,count in peaks:
        height=float(np.median(z[bins==value]));near=np.abs(z-height)<.006
        if near.sum()<max(80,len(z)*.08) or any(abs(height-p)<.012 for p in planes):continue
        planes.append(float(np.median(z[near])))
    if not planes:raise ValueError('No sufficiently observed horizontal support plane')
    # For on-placement choose the upper observed surface, not whichever height
    # band happens to have the most depth pixels (often a lower face/background).
    floor=max(planes)
    floor_mask=np.abs(z-floor)<.006
    floor_points=world[floor_mask]
    _,_,axes=np.linalg.svd(floor_points-np.median(floor_points,axis=0),full_matrices=False)
    tilt_deg=float(np.degrees(np.arccos(np.clip(abs(axes[-1,2]),0.,1.))))
    height_std=float(np.std(floor_points[:,2]))
    if diagnostics is not None:
        diagnostics['support_plane']=dict(selection='highest_observed_surface',candidate_heights_m=sorted(planes,reverse=True),
            tilt_deg=tilt_deg,max_tilt_deg=10.,
            height_std_m=height_std,max_height_std_m=.0035,point_count=len(floor_points))
    if tilt_deg>10. or height_std>.0035:
        raise ValueError('Support plane is too noisy or tilted: '
                         f'tilt={tilt_deg:.2f} deg (max 10), height_std={height_std*1000:.2f} mm (max 3.5)')
    rect=cv2.minAreaRect(floor_points[:,:2].astype(np.float32))
    corners=cv2.boxPoints(rect).astype(float);axis=corners[1]-corners[0]
    if np.linalg.norm(axis)<.02:raise ValueError('Observed support is too narrow')
    axis/=np.linalg.norm(axis);rotation=np.array([[axis[0],-axis[1]],[axis[1],axis[0]]])
    projected=world[:,:2]@rotation;floor_xy=projected[floor_mask]
    low,high=np.quantile(floor_xy,[.01,.99],axis=0)
    # Keep at least ~16 cells across a small observed support. A fixed 8 mm
    # cell erased more than half a block's top when the boundary was eroded.
    # Large surfaces retain the existing 8 mm resolution and grid budget.
    step=float(np.clip(np.min(high-low)/16,.002,.008))
    shape=np.ceil((high-low)/step).astype(int)
    if min(shape)<4 or max(shape)>250:raise ValueError('Observed support outside bounded grid dimensions')
    grid=np.zeros(tuple(shape[::-1]),np.uint8)
    cells=np.floor((floor_xy-low)/step).astype(int)
    valid=np.all((cells>=0)&(cells<shape),axis=1);cells=cells[valid]
    grid[cells[:,1],cells[:,0]]=1
    # Only bridge sub-cell sampling holes, then shrink one full cell at boundaries.
    kernel=np.ones((3,3),np.uint8)
    # Closing is extensive: it must not delete occupied boundary cells. Running
    # it on the unpadded array clips dilation at the image edge and accidentally
    # shrinks the region once here, then a second time in the erosion below.
    padded=np.pad(grid,2)
    grid=cv2.morphologyEx(padded,cv2.MORPH_CLOSE,kernel,
                         borderType=cv2.BORDER_CONSTANT,borderValue=0)[2:-2,2:-2]
    grid=cv2.erode(grid,kernel,borderType=cv2.BORDER_CONSTANT,borderValue=0)
    x0,y0,x1,y1=largest_rectangle(grid.astype(bool))
    inner_low=low+np.array([x0,y0])*step;inner_high=low+np.array([x1,y1])*step
    size=inner_high-inner_low
    if diagnostics is not None:
        diagnostics['support_grid']=dict(resolution_m=step,boundary_inset_m=step,
            measured_extent_m=(high-low).tolist(),usable_extent_m=size.tolist(),
            shape_xy=shape.tolist(),observed_cells=int(len(np.unique(cells,axis=0))),
            supported_cells=int(grid.sum()))
    # region_geometry already requires >1 cm dimensions. Whether a particular
    # payload fits belongs to placement planning, not surface perception.
    if min(size)<=.01:raise ValueError('No contiguous observed placement surface larger than 1 cm')
    center=(inner_low+inner_high)/2
    wall_height=0.;outer_low=inner_low-.005;outer_high=inner_high+.005
    xy=rotation@center;yaw=float(np.arctan2(axis[1],axis[0]))
    region=region_geometry(np.eye(4),float(xy[0]),float(xy[1]),floor,float(size[0]),float(size[1]),yaw,
                           relation,wall_height,.005)
    region.update(geometry_scope='visible_support_conservative_rectangle',
                  outer_bounds_xy_m=[*(outer_low-center).tolist(),*(outer_high-center).tolist()],
                  support_point_count=int(floor_mask.sum()),support_height_std_m=float(np.std(floor_points[:,2])),
                  grid_resolution_m=step)
    # Retain alternative aspect ratios for payload-aware selection by the planner.
    options=[]
    for left,top,right,bottom in rectangle_candidates(grid.astype(bool)):
        option_low=low+np.array([left,top])*step;option_high=low+np.array([right,bottom])*step
        if np.min(option_high-option_low)<=.01:continue
        option_center=(option_low+option_high)/2
        option_xy=rotation@option_center
        option_pose=np.asarray(region['T_footprint_region']).copy();option_pose[:2,3]=option_xy
        options.append(dict(T_footprint_region=option_pose.tolist(),size_xy_m=(option_high-option_low).tolist(),
            outer_bounds_xy_m=[*(np.minimum(outer_low,option_low-.005)-option_center).tolist(),
                               *(np.maximum(outer_high,option_high+.005)-option_center).tolist()]))
    region['usable_rectangle_options']=options
    return region
