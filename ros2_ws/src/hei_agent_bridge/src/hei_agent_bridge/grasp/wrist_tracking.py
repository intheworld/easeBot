"""Run-scoped feature registration against a fixed wrist RGB anchor.

YOLO initializes identity; subsequent corrections use the same physical image
features, never a new mask centroid. No calibration or movement is performed.
"""
import hashlib
from pathlib import Path
import cv2
import numpy as np


def calibration_key(calibration):
    import json
    return hashlib.sha256(json.dumps(calibration,sort_keys=True).encode()).hexdigest()


def plane_points(pixels,calibration,camera,up,height):
    k=np.asarray(calibration['k'],float).reshape(3,3);d=np.asarray(calibration['d'],float)
    xy=cv2.undistortPoints(np.asarray(pixels,float).reshape(-1,1,2),k,d if len(d) else None).reshape(-1,2)
    rays=np.c_[xy,np.ones(len(xy))]@camera[:3,:3].T
    denominator=rays@up
    if np.any(abs(denominator)<.15):raise ValueError('Tracked wrist rays too oblique')
    distance=(height-camera[:3,3]@up)/denominator
    if np.any((distance<=.03)|(distance>=1.5)):raise ValueError('Tracked wrist plane outside working view')
    return camera[:3,3]+rays*distance[:,None]


def create_anchor(image,mask,calibration,camera,obb,delta,path):
    from .wrist import WristTargetError
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    interior=cv2.erode((np.asarray(mask)>0).astype(np.uint8),np.ones((7,7),np.uint8))
    points=cv2.goodFeaturesToTrack(gray,maxCorners=128,qualityLevel=.01,minDistance=4,mask=interior,blockSize=5)
    if points is None or len(points)<8:
        raise WristTargetError(0,'Insufficient object texture for consistent wrist tracking; reacquire')
    pixels=points.reshape(-1,2);up=np.asarray(obb['rotation'],float)[:,2]
    height=float(np.asarray(obb['center_m'])@up+obb['size_m'][2]/2)
    world=plane_points(pixels,calibration,camera,up,height)
    center=np.asarray(obb['center_m'])+delta
    # Lab preserves chromatic texture on dark colored objects which grayscale
    # can confuse with similarly dark cast shadows. Appearance checks below
    # still use luminance patches, not a high score from flat color channels.
    flow_image=cv2.cvtColor(image,cv2.COLOR_BGR2Lab)
    np.savez_compressed(path,gray=gray,flow_image=flow_image,image=image,mask=(np.asarray(mask)>0).astype(np.uint8),
                        camera=camera,pixels=pixels,world=world,center=center,up=up,height=height,
                        calibration_key=calibration_key(calibration))
    return {'anchor':str(path),'features':len(pixels),'method':'fixed_anchor_lk_plane_registration'}


def _track_sparse_translation(image,calibration,camera,obb,anchor,report):
    from .wrist import WristTargetError
    def fail(reason):raise WristTargetError(0,'Wrist feature tracking unavailable: '+reason)
    with np.load(Path(anchor['anchor']),allow_pickle=False) as data:
        gray=data['gray'];pixels=data['pixels'];world=data['world'];center=data['center'];up=data['up'];height=float(data['height'])
        flow_image=data['flow_image'] if 'flow_image' in data else gray
        if str(data['calibration_key'])!=calibration_key(calibration):fail('calibration changed')
    current=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    flow_current=cv2.cvtColor(image,cv2.COLOR_BGR2Lab) if flow_image.ndim==3 else current
    if (gray.shape!=current.shape or pixels.ndim!=2 or pixels.shape[1]!=2 or not 8<=len(pixels)<=128
            or world.shape!=(len(pixels),3) or not np.isfinite(world).all()):fail('invalid anchor')
    # Calibrated projection initializes flow, allowing camera motion without
    # requiring a fresh detector mask or an incremental frame-to-frame chain.
    camera_points=(world-camera[:3,3])@camera[:3,:3]
    if np.any(camera_points[:,2]<=.02):fail('anchor behind camera')
    guess=cv2.projectPoints(camera_points,np.zeros(3),np.zeros(3),np.asarray(calibration['k']).reshape(3,3),
                            np.asarray(calibration['d']) if calibration['d'] else None)[0].astype(np.float32)
    source=pixels.astype(np.float32).reshape(-1,1,2)
    options=dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.001))
    found,status,_=cv2.calcOpticalFlowPyrLK(flow_image,flow_current,source,guess,flags=cv2.OPTFLOW_USE_INITIAL_FLOW,**options)
    if found is None:fail('no forward matches')
    back,back_status,_=cv2.calcOpticalFlowPyrLK(flow_current,flow_image,found,source.copy(),flags=cv2.OPTFLOW_USE_INITIAL_FLOW,**options)
    if back is None:fail('no reverse matches')
    good=status.ravel().astype(bool)&back_status.ravel().astype(bool)
    good &= np.linalg.norm(back.reshape(-1,2)-pixels,axis=1)<=.75
    q=found.reshape(-1,2);h,w=current.shape
    good &= np.isfinite(q).all(1)&(q[:,0]>=3)&(q[:,0]<w-3)&(q[:,1]>=3)&(q[:,1]<h-3)
    ids=np.flatnonzero(good);report.update(anchor_features=len(pixels),forward_backward_matches=len(ids))
    if len(ids)<8:fail('fewer than eight consistent matches')
    # Camera pose and the support plane are known. Fit only the two planar
    # translation degrees of freedom actually used by the correction, instead
    # of estimating an unrelated eight-DOF pixel homography. Consensus uses the
    # existing 2 mm metric residual bound and unchanged 60%/eight-point gates.
    observed=plane_points(q[ids],calibration,camera,up,height)
    offsets=observed-world[ids];offsets-=np.outer(offsets@up,up)
    distances=np.linalg.norm(offsets[:,None]-offsets[None,:],axis=2)
    best=int(np.argmax((distances<=.002).sum(axis=1)))
    inliers=distances[best]<=.002
    for _ in range(3):
        offset=np.median(offsets[inliers],axis=0)
        updated=np.linalg.norm(offsets-offset,axis=1)<=.002
        if not updated.any():break
        inliers=updated
    ids=ids[inliers];offsets=offsets[inliers]
    report.update(registration_model='calibrated_planar_translation',flow_color_space='Lab' if flow_image.ndim==3 else 'gray',
                  inliers=len(ids),metric_inlier_fraction=float(len(ids)/good.sum()))
    if len(ids)<8 or len(ids)<.6*good.sum():fail('inconsistent metric feature motion')
    # Require features distributed across the object, not one tiny surviving patch.
    spread=np.ptp(pixels[ids],axis=0)/np.maximum(np.ptp(pixels,axis=0),1.)
    if min(spread)<.5:fail('features confined to a small patch')
    offset=np.median(offsets,axis=0)
    residual=np.linalg.norm(offsets-offset,axis=1);p90=float(np.quantile(residual,.9))
    report.update(inliers=len(ids),registration_residual_p90_m=p90,feature_span_fraction=spread.tolist())
    if p90>.002:fail('planar registration residual exceeds 2 mm')
    delta=center+offset-np.asarray(obb['center_m']);delta-=up*float(delta@up)
    # Compare tracked patches photometrically after geometry, so occlusion or a
    # different object cannot silently replace an anchor with similar features.
    scores=[]
    for a,b in zip(pixels[ids],q[ids]):
        aa=cv2.getRectSubPix(gray,(9,9),tuple(map(float,a))).astype(float).ravel()
        bb=cv2.getRectSubPix(current,(9,9),tuple(map(float,b))).astype(float).ravel()
        aa-=aa.mean();bb-=bb.mean();scores.append(float(aa@bb/max(np.linalg.norm(aa)*np.linalg.norm(bb),1e-9)))
    ncc=float(np.median(scores));report['patch_ncc_median']=ncc
    if ncc<.8:fail('object appearance no longer matches anchor')
    report.update(method='fixed_anchor_lk_plane_registration',delta_m=delta.tolist(),tracked_center_m=(center+offset).tolist(),status='passed')
    return delta


def track_translation(image,calibration,camera,obb,anchor,report,*,max_distance=.02):
    from .wrist import WristTargetError
    from .wrist_dense import dense_translation
    try:
        return _track_sparse_translation(image,calibration,camera,obb,anchor,report)
    except WristTargetError as exc:
        sparse={**report,'error':str(exc)}
        report.clear();report['sparse_attempt']=sparse
        try:
            return dense_translation(image,calibration,camera,obb,anchor,report,max_distance)
        except WristTargetError as dense_error:
            report['dense_error']=str(dense_error)
            if anchor.get('keyframe'):
                original_error=dict(report)
                recent={}
                try:
                    delta=dense_translation(image,calibration,camera,obb,anchor['keyframe'],recent,max_distance)
                except WristTargetError as keyframe_error:
                    report['keyframe_attempt']={**recent,'error':str(keyframe_error)}
                else:
                    report.clear();report.update(recent,original_attempt=original_error,
                        keyframe_recovery=True,keyframe_anchor=anchor['keyframe']['anchor'])
                    return delta
            raise WristTargetError(0,f'Sparse and dense registration unavailable; sparse: {exc}; dense: {dense_error}') from dense_error
