"""Bounded local quadratic interpolation of enclosed invalid depth returns.

Measured values are immutable. A complete surrounding ring must fit under the
same residual and area limits as the earlier planar interpolation policy.
"""
import cv2
import numpy as np


def features(xy):
    x,y=np.asarray(xy).T
    return np.c_[np.ones(len(x)),x,y,x*x,x*y,y*y]


def recover_surface(depth,mask,intrinsic,transform,floor):
    depth=np.asarray(depth,float);mask=np.asarray(mask,bool);k=np.asarray(intrinsic,float).reshape(3,3)
    if depth.ndim!=2 or mask.shape!=depth.shape:raise ValueError('Invalid support depth mask')
    v,u=np.indices(depth.shape)
    rays=np.stack(((u-k[0,2])/k[0,0],(v-k[1,2])/k[1,1],np.ones(depth.shape)),axis=-1)@transform[:3,:3].T
    origin=transform[:3,3];valid=np.isfinite(depth)&(depth>0)
    count,labels,stats,_=cv2.connectedComponentsWithStats((mask&~valid).astype(np.uint8),8)
    points=[];reports=[];surface_mask=np.zeros(depth.shape,bool);used=0;budget=int(mask.sum()*.10)
    for label in range(1,count):
        x,y,w,h,area=map(int,stats[label])
        if area<4:continue
        report=dict(pixels=area,accepted=False);reports.append(report)
        if used+area>budget or x<2 or y<2 or x+w>depth.shape[1]-2 or y+h>depth.shape[0]-2:continue
        hole=labels==label;ring=cv2.dilate(hole.astype(np.uint8),np.ones((13,13),np.uint8)).astype(bool)&~hole
        good=ring&mask&valid
        if not mask[ring].all() or good.sum()<30 or good.sum()<.95*ring.sum():continue
        measured=rays[good]*depth[good,None]+origin
        if np.mean(abs(measured[:,2]-floor)<.006)<.25:continue
        center=np.mean(measured[:,:2],axis=0);xy=measured[:,:2]-center
        scale=np.max(np.ptp(xy,axis=0))
        if scale<.001:continue
        design=features(xy/scale);a,_,rank,_=np.linalg.lstsq(design,measured[:,2],rcond=None)
        error=design@a-measured[:,2];rms=float(np.sqrt(np.mean(error**2)));p95=float(np.quantile(abs(error),.95))
        report.update(plane_rms_m=rms,residual_p95_m=p95,model='local_quadratic')
        if rank<6 or np.linalg.cond(design)>1000 or rms>.0035 or p95>.006:continue
        directions=rays[hole]
        with np.errstate(divide='ignore',invalid='ignore'):
            distance=(floor-origin[2])/directions[:,2]
            for _ in range(8):
                p=origin+distance[:,None]*directions;xx,yy=((p[:,:2]-center)/scale).T
                predicted=features(np.c_[xx,yy])@a
                gx=(a[1]+2*a[3]*xx+a[4]*yy)/scale;gy=(a[2]+a[4]*xx+2*a[5]*yy)/scale
                derivative=directions[:,2]-gx*directions[:,0]-gy*directions[:,1]
                distance-=(p[:,2]-predicted)/derivative
        p=origin+distance[:,None]*directions
        residual=abs(p[:,2]-features((p[:,:2]-center)/scale)@a)
        # Stay inside the observed XY hull and height envelope plus residual bound.
        hull=cv2.convexHull(measured[:,:2].astype(np.float32))
        enclosed=np.array([cv2.pointPolygonTest(hull,tuple(map(float,q[:2])),False)>=0 if np.isfinite(q).all() else False for q in p])
        accept=enclosed&np.isfinite(p).all(axis=1)&(distance>0)&(residual<.0001)
        accept&=(p[:,2]>=measured[:,2].min()-.006)&(p[:,2]<=measured[:,2].max()+.006)
        # Curvature is checked through residuals; retain the original patch-average tilt bound.
        linear=np.linalg.lstsq(np.c_[xy,np.ones(len(xy))],measured[:,2],rcond=None)[0]
        if 1/np.sqrt(1+linear[:2]@linear[:2])<.98:continue
        good_indices=np.flatnonzero(good)
        surface_mask.flat[good_indices[abs(error)<=.006]]=True
        if accept.any():
            points.append(p[accept]);used+=area;report.update(accepted=True,inferred_points=int(accept.sum()))
    return (np.concatenate(points) if points else np.empty((0,3))),dict(patches=reports,inferred_support_pixels=sum(len(p) for p in points),budget_pixels=budget),surface_mask


def measured_surface(points, training_mask, floor, *, max_tilt_deg=5.):
    """Explain valid returns using the observed bottom, independently of holes.

    Initialize from the original bottom slab; bounded inlier refinement must
    continue to explain those fixed anchors under the same error/tilt limits.
    Classification is restricted to its XY convex hull and measured height
    envelope. Outside/unsupported returns retain the original occupied/unknown
    classification. This function neither interpolates depth nor edits points.
    """
    points=np.asarray(points,float);training_mask=np.asarray(training_mask,bool)
    accepted=np.zeros(len(points),bool)
    seed=training_mask&np.isfinite(points).all(axis=1)&(abs(points[:,2]-floor)<.006)
    report={'model':'anchored_quadratic','seed_points':int(seed.sum()),'accepted':False}
    if seed.sum()<80:return accepted,report
    measured=points[seed];center=measured[:,:2].mean(axis=0)
    scale=float(np.max(np.ptp(measured[:,:2],axis=0)))
    if scale<.01:return accepted,report
    xy=(measured[:,:2]-center)/scale;design=features(xy)
    a,_,rank,_=np.linalg.lstsq(design,measured[:,2],rcond=None)
    residual=design@a-measured[:,2]
    rms=float(np.sqrt(np.mean(residual**2)));p95=float(np.quantile(abs(residual),.95))
    tilt=float(np.degrees(np.arctan(np.linalg.norm(a[1:3]/scale))))
    report.update(rms_m=rms,residual_p95_m=p95,tilt_deg=tilt)
    if rank<6 or np.linalg.cond(design)>1000 or rms>.0035 or p95>.006 or tilt>max_tilt_deg:
        return accepted,report
    # Refine the same continuous surface using inliers, while keeping every
    # original bottom anchor and the existing residual/tilt requirements.
    all_design=features((points[:,:2]-center)/scale)
    for _ in range(3):
        inliers=training_mask&(abs(points[:,2]-all_design@a)<=.006)
        candidate,_,rank,_=np.linalg.lstsq(all_design[inliers],points[inliers,2],rcond=None)
        anchor_error=design@candidate-measured[:,2]
        fit_error=all_design[inliers]@candidate-points[inliers,2]
        candidate_tilt=float(np.degrees(np.arctan(np.linalg.norm(candidate[1:3]/scale))))
        if (rank<6 or np.sqrt(np.mean(fit_error**2))>.0035 or
            np.quantile(abs(anchor_error),.95)>.006 or candidate_tilt>max_tilt_deg):break
        a=candidate
    residual=design@a-measured[:,2]
    report.update(rms_m=float(np.sqrt(np.mean(residual**2))),residual_p95_m=float(np.quantile(abs(residual),.95)),
                  coefficients=a.tolist(),tilt_deg=float(np.degrees(np.arctan(np.linalg.norm(a[1:3]/scale)))))
    predicted=features((points[:,:2]-center)/scale)@a
    candidate=np.flatnonzero(np.isfinite(points).all(axis=1)&(abs(points[:,2]-predicted)<=.006)&
        (predicted>=measured[:,2].min()-.006)&(predicted<=measured[:,2].max()+.006))
    hull=cv2.convexHull(measured[:,:2].astype(np.float32))
    accepted[candidate]=[cv2.pointPolygonTest(hull,tuple(map(float,points[i,:2])),False)>=0 for i in candidate]
    report.update(accepted=True,explained_points=int(accepted.sum()),coefficients=a.tolist(),
                  center_xy_m=center.tolist(),scale_m=scale)
    return accepted,report
