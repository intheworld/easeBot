"""One-hop appearance recovery, always rooted in the original accepted target.

Only observations registered directly to the original anchor can supply a new
keyframe. A keyframe recovery never updates the keyframe again. Its target
center comes from the original registration, never a new detector centroid.
"""
import cv2
import numpy as np
from .wrist_tracking import plane_points, create_anchor, calibration_key


def create_keyframe(image, calibration, camera, obb, delta, original, path):
    with np.load(original['anchor'], allow_pickle=False) as data:
        if str(data['calibration_key']) != calibration_key(calibration):
            raise ValueError('Keyframe calibration changed')
        mask=data['mask'];origin_camera=data['camera'];center=data['center']
        up=data['up'];height=float(data['height'])
    h,w=mask.shape
    k=np.asarray(calibration['k'],float).reshape(3,3);d=np.asarray(calibration['d'],float)
    undistorted={**calibration,'d':[]}
    support=cv2.undistort(mask,k,d)
    corners=np.array([[0,0],[w-1,0],[w-1,h-1],[0,h-1]],np.float32)
    offset=np.asarray(obb['center_m'])+delta-center
    world=plane_points(corners,undistorted,origin_camera,up,height)+offset
    cp=(world-camera[:3,3])@camera[:3,:3]
    if np.any(cp[:,2]<=.02):raise ValueError('Keyframe plane behind camera')
    pixels=cv2.projectPoints(cp,np.zeros(3),np.zeros(3),k,None)[0].reshape(-1,2).astype(np.float32)
    warped=cv2.warpPerspective(support,cv2.getPerspectiveTransform(corners,pixels),(w,h),flags=cv2.INTER_NEAREST)
    # Map each distorted output pixel back to the undistorted support image.
    yy,xx=np.indices((h,w))
    maps=cv2.undistortPoints(np.c_[xx.ravel(),yy.ravel()].astype(np.float32)[:,None],k,d if len(d) else None,P=k).reshape(h,w,2)
    warped=cv2.remap(warped,maps[:,:,0],maps[:,:,1],cv2.INTER_NEAREST)
    return create_anchor(image,warped,calibration,camera,obb,delta,path)
