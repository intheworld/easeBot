"""Shared metric coverage used by placement sampling and collision geometry."""
import numpy as np


def unknown_fraction(region):
    value=region.get('max_unknown_support_fraction',0.)
    if type(value) not in (int,float) or not np.isfinite(value) or not 0<=value<=1:
        raise ValueError('Invalid max_unknown_support_fraction')
    return float(value)


def coverage(region):
    model=region['spatial_model']
    if model.get('schema') not in ('hei.container_space/v1','hei.container_space/v2'):
        raise ValueError('Unsupported container spatial model')
    data=model['coverage'];grid=np.asarray(data['cells']);step=float(data['resolution_m'])
    origin=np.asarray(data['origin_xy_m'],float)
    if (grid.ndim!=2 or grid.size>62500 or not np.isin(grid,[0,1,2,3]).all()
            or list(grid.shape[::-1])!=data['shape_xy'] or origin.shape!=(2,)
            or not np.isfinite(origin).all() or not .001<=step<=.05):
        raise ValueError('Invalid container coverage grid')
    return grid,origin,step


def supported_centers(region,half_extent,available,polygon=None,*,diverse=False):
    """Test the actual padded convex footprint against unsupported cell areas."""
    grid,origin,step=coverage(region)
    if not np.any(np.isin(grid,[1,3])):
        return
    if polygon is None:
        polygon=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*half_extent
    polygon=np.asarray(polygon,float)
    edges=np.roll(polygon,-1,axis=0)-polygon
    normals=np.c_[-edges[:,1],edges[:,0]]
    normals=normals[np.linalg.norm(normals,axis=1)>1e-10]
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    axes=np.vstack((np.eye(2),normals))
    poly_min=(polygon@axes.T).min(axis=0);poly_max=(polygon@axes.T).max(axis=0)
    allowance=unknown_fraction(region)
    by,bx=np.nonzero(~np.isin(grid,[1,3]))
    labels=grid[by,bx]
    blocked=origin+(np.c_[bx,by]+.5)*step
    box_projection=blocked@axes.T;radius=np.abs(axes).sum(axis=1)*step/2
    # Sub-cell sampling prevents narrow valid intervals falling between centers.
    counts=np.minimum(129,2*np.ceil(available/(step/8)).astype(int)+1)
    xx,yy=np.meshgrid(np.linspace(-available[0],available[0],max(1,counts[0])),
                      np.linspace(-available[1],available[1],max(1,counts[1])))
    centers=np.c_[xx.ravel(),yy.ravel()]
    order=np.argsort(np.sum(centers**2,axis=1),kind='stable')
    if diverse:
        # Visit widely separated locations before spending the budget on nearby
        # sub-cell samples. All locations retain the identical support test.
        anchors=np.array([[0,0],[-1,-1],[1,1],[-1,1],[1,-1],[-1,0],[1,0],[0,-1],[0,1]],float)*available
        # Coarse-to-fine anchors cover the whole region before spending the
        # budget on sub-millimeter neighbors around the first feasible point.
        for count in (5,9):
            ax,ay=np.meshgrid(np.linspace(-1,1,count),np.linspace(-1,1,count))
            anchors=np.vstack((anchors,np.c_[ax.ravel(),ay.ravel()]*available))
        first=list(dict.fromkeys(int(np.argmin(np.sum((centers-a)**2,axis=1))) for a in anchors))
        order=np.r_[first,order[~np.isin(order,first)]]
    # Count the full area of every intersecting unknown cell. This is an upper
    # bound on unknown footprint area, never an underestimate at polygon edges.
    area=abs(np.sum(polygon[:,0]*np.roll(polygon[:,1],-1)-polygon[:,1]*np.roll(polygon[:,0],-1)))/2
    if not np.isfinite(area) or area<=0:raise ValueError('Invalid support footprint polygon')
    ordered=centers[order]
    # Bounded vectorized SAT preserves the original candidate order and exact
    # occupied-cell rejection without a Python loop over every sampled center.
    batch=max(1,min(128,1000000//max(1,len(blocked))))
    for start in range(0,len(ordered),batch):
        candidates=ordered[start:start+batch];shift=candidates@axes.T
        overlap=np.ones((len(candidates),len(blocked)),bool)
        for axis in range(len(axes)):
            overlap &= ((box_projection[None,:,axis]+radius[axis]>poly_min[axis]+shift[:,None,axis]+1e-10)&
                        (box_projection[None,:,axis]-radius[axis]<poly_max[axis]+shift[:,None,axis]-1e-10))
        allowed=~np.any(overlap[:,labels==2],axis=1)
        unknown_area=overlap[:,labels==0].sum(axis=1)*step*step
        allowed &= unknown_area<=allowance*area+1e-12
        for center in candidates[allowed]:yield center


def exclusion_boxes(region):
    """Exact row-run rectangles; unknown is an exclusion, not an observed wall.

    Keep sub-boxes in one scene entry so the object-count limit continues to
    describe objects, not the tessellation density of one observed container.
    """
    grid,origin,step=coverage(region);model=region['spatial_model'];rectangles=[]
    tops=model['coverage'].get('occupied_top_z_m')
    if tops is not None:
        tops=np.asarray(tops,float)
        if tops.shape!=grid.shape or not np.isfinite(tops).all():raise ValueError('Invalid occupied height map')
    for label,name,key in ((0,'unknown','unknown_height_m'),(2,'occupied','occupied_height_m')):
        # Under the explicit partial-observation policy, missing depth is not a
        # measured solid obstacle. Footprint uncertainty is checked separately.
        if label==0 and unknown_fraction(region)>0:continue
        active={}
        for y in range(grid.shape[0]+1):
            row=(grid[y]==label) if y<grid.shape[0] else np.zeros(grid.shape[1],bool)
            edges=np.diff(np.r_[False,row,False].astype(int))
            runs=set(zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)))
            for run,start in list(active.items()):
                if run not in runs:
                    rectangles.append((name,key,run[0],start,run[1],y));del active[run]
            for run in runs:active.setdefault(run,y)
    if len(rectangles)>4096:raise ValueError('Container coverage too fragmented; recapture')
    boxes=[];size=np.asarray(region['size_xy_m'])
    for name,key,x0,y0,x1,y1 in rectangles:
        low=np.maximum(origin+np.array([x0,y0])*step,-size/2)
        high=np.minimum(origin+np.array([x1,y1])*step,size/2)
        height=float(model[key])
        if name=='occupied' and tops is not None:
            height=float(np.max(tops[y0:y1,x0:x1])-region['T_footprint_region'][2][3])
            # Returns below the collision floor are already enclosed by it.
            if height<=0:continue
        if not np.isfinite(height) or not 0<height<=2:raise ValueError('Invalid coverage exclusion height')
        if np.any(high<=low):continue
        transform=np.eye(4);transform[:3,3]=[*(low+high)/2,height/2]
        boxes.append(dict(kind=name,size_m=[*(high-low),height],T_region_box=transform.tolist()))
    return boxes
