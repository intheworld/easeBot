"""Calibrated whole-target image registration when sparse texture is lost.

The original target template remains fixed. Match appearance, competing peaks,
reverse registration and independent luminance location before estimating any
planar displacement. This module never dispatches motion or changes calibration.
"""
import cv2
import numpy as np

from .wrist_tracking import plane_points,calibration_key


def _peak(scores):
    if not np.isfinite(scores).all():raise ValueError('Non-finite dense correlation')
    _,score,_,where=cv2.minMaxLoc(scores)
    return float(score),np.array(where,int)


def dense_translation(image,calibration,camera,obb,anchor,report,max_distance):
    from .wrist import WristTargetError
    def fail(reason):raise WristTargetError(0,'Wrist dense registration unavailable: '+reason)
    with np.load(anchor['anchor'],allow_pickle=False) as data:
        if str(data['calibration_key'])!=calibration_key(calibration):fail('calibration changed')
        if not all(k in data for k in ('image','mask','camera')):fail('anchor has no dense image evidence')
        initial=data['image'];mask=data['mask'];origin_camera=data['camera'];up=data['up'];height=float(data['height']);center=data['center']
    if initial.shape!=image.shape or mask.shape!=image.shape[:2] or not np.isfinite(camera).all():fail('invalid anchor/image')
    h,w=mask.shape;k=np.asarray(calibration['k'],float).reshape(3,3);d=np.asarray(calibration['d'],float)
    old=cv2.undistort(initial,k,d);current=cv2.undistort(image,k,d)
    support=cv2.undistort((mask>0).astype(np.uint8)*255,k,d)>127
    x,y,width,length=cv2.boundingRect(support.astype(np.uint8));pad=6
    if min(width,length)<8:fail('template too small')
    corners=np.array([[x-pad,y-pad],[x+width+pad,y-pad],
                      [x+width+pad,y+length+pad],[x-pad,y+length+pad]],np.float32)
    if (corners.min(0)<0).any() or (corners.max(0)>[w-1,h-1]).any():fail('anchor clipped')
    undistorted={**calibration,'d':[]}
    world=plane_points(corners,undistorted,origin_camera,up,height)
    camera_points=(world-camera[:3,3])@camera[:3,:3]
    if np.any(camera_points[:,2]<=.02):fail('template behind camera')
    destination=cv2.projectPoints(camera_points,np.zeros(3),np.zeros(3),k,None)[0].reshape(-1,2).astype(np.float32)
    homography=cv2.getPerspectiveTransform(corners,destination)
    warped=cv2.warpPerspective(old,homography,(w,h))
    warped_mask=cv2.warpPerspective(support.astype(np.uint8),homography,(w,h),flags=cv2.INTER_NEAREST)
    x,y,width,length=cv2.boundingRect(warped_mask)
    x-=pad;y-=pad;width+=2*pad;length+=2*pad
    if min(x,y)<0 or x+width>=w or y+length>=h:fail('projected template clipped')
    # A mostly background template would silently track the table. Restrict this
    # fallback to a target-dominated patch; sparse tracking remains independent.
    if np.count_nonzero(warped_mask[y:y+length,x:x+width])<.5*width*length:
        fail('template does not isolate the target')
    vv,uu=np.nonzero(support);anchor_pixel=np.array([[uu.mean(),vv.mean()]],np.float32)
    predicted=cv2.perspectiveTransform(anchor_pixel[:,None],homography).reshape(1,2)
    point=plane_points(anchor_pixel,undistorted,origin_camera,up,height)[0]
    # Bound the image search by the existing metric association range.
    axes=np.asarray(obb['rotation'])[:,:2]
    extremes=point+np.array([[-1,-1],[1,-1],[1,1],[-1,1]])@axes.T*max_distance
    cp=(extremes-camera[:3,3])@camera[:3,:3]
    pixels=cv2.projectPoints(cp,np.zeros(3),np.zeros(3),k,None)[0].reshape(-1,2)
    radius=np.maximum(4,np.ceil(np.max(abs(pixels-predicted),axis=0)).astype(int)+2)
    rx,ry=np.maximum([0,0],np.array([x,y])-radius)
    ex,ey=np.minimum([w,h],np.array([x+width,y+length])+radius)
    template=warped[y:y+length,x:x+width];search=current[ry:ey,rx:ex]
    if max(np.std(template.astype(float),axis=(0,1)))<1:fail('uninformative template')
    scores=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
    score,where=_peak(scores);shift=where+[rx-x,ry-y]
    report.update(method='fixed_anchor_dense_registration',dense_score=score,template_size_px=[width,length],search_radius_px=radius.tolist())
    if score<.8:fail('target appearance does not match anchor')
    if (where==0).any() or where[0]==scores.shape[1]-1 or where[1]==scores.shape[0]-1:
        fail('best match at search boundary')
    selected=plane_points(predicted+shift,undistorted,camera,up,height)[0]
    # Separate local maxima identify competing objects, rather than rejecting
    # nearby samples on the smooth shoulder of the same correlation peak.
    maxima=cv2.dilate(scores,np.ones((5,5),np.uint8))
    py,px=np.nonzero((scores>=.8)&(scores>=maxima-1e-7))
    if len(px)>512:fail('ambiguous correlation plateau')
    alternatives=plane_points(predicted+np.c_[px+rx-x,py+ry-y],undistorted,camera,up,height)
    competitors=np.linalg.norm(alternatives-selected,axis=1)>.002
    report['competing_peaks']=int(competitors.sum())
    if competitors.any():fail('multiple plausible target locations')
    gray_template=cv2.cvtColor(template,cv2.COLOR_BGR2GRAY)
    gray_search=cv2.cvtColor(search,cv2.COLOR_BGR2GRAY)
    gray_score,gray_where=_peak(cv2.matchTemplate(gray_search,gray_template,cv2.TM_CCOEFF_NORMED))
    gray_point=plane_points(predicted+gray_where+[rx-x,ry-y],undistorted,camera,up,height)[0]
    independent_error=float(np.linalg.norm(gray_point-selected));report.update(luminance_score=gray_score,color_luminance_difference_m=independent_error)
    if independent_error>.002:fail('color and luminance disagree on location')
    patch=search[where[1]:where[1]+length,where[0]:where[0]+width]
    reverse_score,reverse=_peak(cv2.matchTemplate(warped[ry:ey,rx:ex],patch,cv2.TM_CCOEFF_NORMED))
    reverse_point=plane_points(predicted+reverse+[rx-x,ry-y],undistorted,camera,up,height)[0]
    expected_point=plane_points(predicted,undistorted,camera,up,height)[0]
    reverse_error=float(np.linalg.norm(reverse_point-expected_point))
    report.update(reverse_score=reverse_score,reverse_error_m=reverse_error)
    if reverse_score<.8 or reverse_error>.002:fail('reverse registration inconsistent')
    offset=selected-point;offset-=up*(offset@up)
    delta=center+offset-np.asarray(obb['center_m']);delta-=up*(delta@up)
    if np.linalg.norm(delta)>max_distance:fail('tracked target outside correction bounds')
    report.update(delta_m=delta.tolist(),tracked_center_m=(center+offset).tolist(),status='passed')
    return delta
