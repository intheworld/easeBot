"""Calibrated wrist RGB silhouette registration; no ROS or hardware access.

The front OBB supplies size, orientation and height. RGB only estimates planar
translation by comparing the observed silhouette to that projected 3D box.
It neither measures new depth nor recalibrates the camera or verifies a grasp.
"""
from itertools import product
import json
from pathlib import Path

import cv2
import numpy as np

from .transforms import validate_transforms


DEFAULTS = dict(enabled=False, calibration_file='', tolerance_m=.005, max_step_m=.005,
                max_total_m=.02, max_updates=5, settle_sec=1.,
                frame_timeout_sec=3., compute_timeout_sec=45.,
                stable_frames=3, min_sharpness=0., max_frame_difference=3.,
                joint_stable_sec=.4, joint_timeout_sec=6., joint_span_rad=.003,
                joint_sample_skew_sec=.05, capture_retries=2)


def refinement_config(value):
    if type(value) is not dict or set(value)-set(DEFAULTS):
        raise ValueError('Invalid wrist_refinement configuration')
    config = {**DEFAULTS, **value}
    if type(config['enabled']) is not bool or type(config['calibration_file']) is not str:
        raise ValueError('Invalid wrist refinement enable/calibration_file')
    if type(config['max_updates']) is not int or not 1 <= config['max_updates'] <= 10:
        raise ValueError('Invalid wrist refinement max_updates')
    if type(config['stable_frames']) is not int or not 2 <= config['stable_frames'] <= 10:
        raise ValueError('Invalid wrist refinement stable_frames')
    if type(config['capture_retries']) is not int or not 0<=config['capture_retries']<=3:
        raise ValueError('Invalid wrist refinement capture_retries')
    limits = dict(joint_stable_sec=(.1,2.), joint_timeout_sec=(1.,10.), joint_span_rad=(.0005,.005),
                  joint_sample_skew_sec=(.005,.1), min_sharpness=(0.,1000.), max_frame_difference=(.1,20.), tolerance_m=(.0005,.005), max_step_m=(.001,.01), max_total_m=(.001,.03),
                  settle_sec=(.1,2.), frame_timeout_sec=(.5,10.), compute_timeout_sec=(1.,90.))
    for key,(low,high) in limits.items():
        v=config[key]
        if type(v) not in (float,int) or not np.isfinite(v) or not low<=v<=high:
            raise ValueError(f'Invalid wrist refinement {key}')
    if config['joint_timeout_sec']<=config['joint_stable_sec']:
        raise ValueError('Joint timeout must exceed the stability window')
    if not config['tolerance_m'] <= config['max_step_m'] <= config['max_total_m']:
        raise ValueError('Wrist refinement requires tolerance <= step <= total')
    return config


def frame_quality(encoded,previous=None):
    """Heuristic image gate, not a guarantee of focus or exposure timestamp."""
    image=cv2.imdecode(np.frombuffer(encoded,dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
    if image is None:raise ValueError('Cannot decode wrist RGB frame')
    h,w=image.shape
    roi=image[h//10:4*h//5,w//10:9*w//10]
    if min(roi.shape)<16:raise ValueError('Wrist RGB frame is too small')
    sharpness=float(cv2.Laplacian(roi,cv2.CV_64F).var())
    difference=None if previous is None or previous.shape!=roi.shape else float(cv2.absdiff(roi,previous).mean())
    return roi,dict(sharpness=sharpness,frame_difference=difference)


def load_calibration(path):
    if not path:
        raise ValueError('Wrist refinement requires measured right-wrist intrinsics and T_tcp_camera; '
                         'set right_grasp.wrist_refinement.calibration_file')
    value=json.loads(Path(path).expanduser().read_text())
    for key,expected in [('schema','hei.wrist_camera/v1'),('camera','right_wrist'),
                         ('frame_id','right_wrist_camera_optical_frame'),('tool_frame','a_right_tcp')]:
        if value.get(key)!=expected:raise ValueError(f'Wrist calibration {key} must be {expected}')
    if any(type(value.get(k)) is not int or not 1<=value[k]<=4096 for k in ('width','height')):
        raise ValueError('Wrist calibration needs measured image width/height')
    k=np.asarray(value.get('k'),dtype=float)
    d=np.asarray(value.get('d'),dtype=float)
    if k.shape!=(9,) or not np.isfinite(k).all() or k[0]<=0 or k[4]<=0 or not np.allclose(k[6:],[0,0,1]):
        raise ValueError('Invalid wrist camera matrix k')
    if d.ndim!=1 or len(d) not in (0,4,5,8,12,14) or not np.isfinite(d).all():
        raise ValueError('Invalid wrist distortion coefficients')
    if value.get('distortion_model') not in ('plumb_bob','rational_polynomial'):
        raise ValueError('Unsupported wrist distortion model')
    validate_transforms(np.asarray(value.get('T_tcp_camera'),dtype=float)[None],'camera optical to TCP')
    return value


def ray_on_plane(pixel,calibration,base_camera,up,height):
    k=np.asarray(calibration['k']).reshape(3,3)
    d=np.asarray(calibration['d'])
    xy=cv2.undistortPoints(np.asarray(pixel,dtype=float).reshape(1,1,2),k,d if len(d) else None).ravel()
    ray=base_camera[:3,:3] @ np.r_[xy,1.]
    denominator=float(up @ ray)
    if abs(denominator)<.15:raise ValueError('Wrist view is too oblique for planar correction')
    distance=(height-float(up @ base_camera[:3,3]))/denominator
    if not .03<distance<1.5:raise ValueError('Target plane is behind/outside the wrist camera working view')
    return base_camera[:3,3]+distance*ray


def projected_silhouette(obb,base_camera,calibration):
    center,size,rotation=(np.asarray(obb[k],dtype=float) for k in ('center_m','size_m','rotation'))
    corners=np.array(list(product((-1.,1.),repeat=3)))*size/2
    corners=corners@rotation.T+center
    camera=(corners-base_camera[:3,3]) @ base_camera[:3,:3]
    if np.any(camera[:,2]<=.02):raise ValueError('OBB is behind/too close to wrist camera')
    pixels=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),np.asarray(calibration['k']).reshape(3,3),
                             np.asarray(calibration['d']) if calibration['d'] else None)[0].reshape(-1,2)
    if (pixels.min(axis=0)<3).any() or (pixels.max(axis=0)>[calibration['width']-4,calibration['height']-4]).any():
        raise ValueError('Predicted target is outside the complete wrist view')
    hull=cv2.convexHull(pixels.astype(np.float32))
    moments=cv2.moments(hull)
    if moments['m00']<30:raise ValueError('Projected wrist target is too small')
    return np.array([moments['m10'],moments['m01']])/moments['m00'],moments['m00']



class WristTargetError(ValueError):
    def __init__(self,count,reason=None):
        self.count=count
        message=f'Wrist target missing, ambiguous or outside correction bounds ({count} plausible masks)'
        super().__init__(message+('; '+reason if reason else ''))

def estimate_translation(obb,base_tcp,calibration,results,target,max_distance,diagnostics=None,reference_area_fraction=None):
    from .common_utils import tensor_to_numpy
    base_camera=base_tcp @ np.asarray(calibration['T_tcp_camera'])
    expected,expected_area=projected_silhouette(obb,base_camera,calibration)
    rotation=np.asarray(obb['rotation']);up=rotation[:,2]
    height=float(np.asarray(obb['center_m']) @ up+obb['size_m'][2]/2)
    reference=ray_on_plane(expected,calibration,base_camera,up,height)
    if reference_area_fraction is not None and (not np.isfinite(reference_area_fraction) or not .35<=reference_area_fraction<=2.5):
        raise ValueError('Invalid wrist silhouette reference')
    candidates=[]
    diagnostics={} if diagnostics is None else diagnostics
    diagnostics.update(expected_pixel=expected.tolist(),expected_area_px=float(expected_area),detections=[])
    for result in results:
        boxes,masks=getattr(result,'boxes',None),getattr(result,'masks',None)
        if boxes is None or masks is None:continue
        cls,conf,segments=(tensor_to_numpy(v) for v in (boxes.cls,boxes.conf,masks.data))
        for i,(category,score) in enumerate(zip(cls.ravel(),conf.ravel())):
            if result.names[int(category)]!=target or not np.isfinite(score) or score<.25 or i>=len(segments):continue
            segment=np.asarray(segments[i],dtype=np.float32)
            if segment.shape!=(calibration['height'],calibration['width']):
                raise ValueError('Wrist masks must use original image coordinates (retina_masks=True)')
            mask=(segment>.5).astype(np.uint8)
            count=int(mask.sum())
            if count<30:continue
            y,x=np.nonzero(mask)
            if x.min()<3 or y.min()<3 or x.max()>=calibration['width']-3 or y.max()>=calibration['height']-3:continue
            components,_,stats,_=cv2.connectedComponentsWithStats(mask)
            if components<2 or stats[1:,cv2.CC_STAT_AREA].max()<.9*count:continue
            pixel=np.array([x.mean(),y.mean()])
            point=ray_on_plane(pixel,calibration,base_camera,up,height)
            delta=point-reference;delta-=up*float(delta @ up)
            fraction=count/expected_area
            consistent=reference_area_fraction is None or .85<=fraction/reference_area_fraction<=1.18
            geometry_ok=.35<=fraction<=2.5 and np.linalg.norm(delta)<=max_distance
            plausible=geometry_ok and consistent
            diagnostics['detections'].append(dict(confidence=float(score),pixel=pixel.tolist(),
                area_px=count,area_fraction=float(fraction),reference_area_fraction=reference_area_fraction,
                silhouette_consistent=bool(consistent),geometry_plausible=bool(geometry_ok),delta_m=delta.tolist(),plausible=bool(plausible)))
            if plausible:candidates.append((delta,mask))
    if len(candidates)!=1:
        partial=any(v['geometry_plausible'] and not v['silhouette_consistent'] for v in diagnostics['detections'])
        raise WristTargetError(len(candidates),'Silhouette area changed; possible partial-object mask, reacquire' if partial else None)
    diagnostics['selected_area_fraction']=next(v['area_fraction'] for v in diagnostics['detections'] if v['plausible'])
    return candidates[0][0],candidates[0][1],diagnostics


def bounded_hover(current,origin,estimated,up,config):
    """Saturate the commanded planar target; retain measured error for convergence."""
    up=np.asarray(up,float)
    offset=np.asarray(estimated)[:3,3]-np.asarray(origin)[:3,3]
    offset-=up*float(offset@up)
    requested_norm=float(np.linalg.norm(offset));limit=config['max_total_m']
    limited=offset*min(1.,limit/max(requested_norm,1e-12))
    error=np.asarray(estimated)[:3,3]-current[:3,3]
    error-=up*float(error@up)
    commanded_error=np.asarray(origin)[:3,3]+limited-current[:3,3]
    commanded_error-=up*float(commanded_error@up)
    traveled=current[:3,3]-np.asarray(origin)[:3,3]
    traveled-=up*float(traveled@up)
    converged=bool(np.linalg.norm(error)<=config['tolerance_m'] and np.linalg.norm(traveled)<=limit+1e-12)
    pre=current.copy()
    if not converged:
        pre[:3,3]+=commanded_error*min(1.,config['max_step_m']/max(float(np.linalg.norm(commanded_error)),1e-12))
    traveled=pre[:3,3]-np.asarray(origin)[:3,3]
    traveled-=up*float(traveled@up)
    clipped=traveled*min(1.,limit/max(float(np.linalg.norm(traveled)),1e-12))
    pre[:3,3]+=clipped-traveled
    if np.linalg.norm(pre[:3,3]-current[:3,3])>config['max_step_m']+1e-9:
        raise ValueError('Measured hover is too far outside the correction envelope for a bounded step')
    return pre,converged,dict(alignment_error_m=error.tolist(),
        requested_hover_offset_m=offset.tolist(),limited_hover_offset_m=limited.tolist(),
        target_offset_saturated=bool(requested_norm>limit),commanded_alignment_error_m=commanded_error.tolist(),
        applied_step_m=(pre[:3,3]-current[:3,3]).tolist(),hover_offset_m=clipped.tolist())


def refine_request(request,detector=None):
    """Estimate a bounded planar target update and prevalidate the next motion."""
    from copy import deepcopy
    from .kinematics import RightArmFK
    from .planner import RightArmPlanner
    from .validation import validate_plan
    config=refinement_config(request['profile']['wrist_refinement'])
    calibration=load_calibration(config['calibration_file'])
    directory=Path(request['directory'])
    image=cv2.imread(str(directory/'rgb.jpg'))
    if image is None or image.shape[:2]!=(calibration['height'],calibration['width']):
        raise ValueError('Wrist RGB resolution differs from its calibration')
    fk=RightArmFK(request['urdf'])
    state=request['state'];current=fk(state['right'])
    reference=deepcopy(request['reference_plan'])
    origin=request['origin_plan']
    up=np.asarray(reference['target_obb']['rotation'])[:,2]
    expected_pre=np.asarray(reference['T_pregrasp'])
    if abs(float((current[:3,3]-expected_pre[:3,3])@up))>.005:
        raise ValueError('Wrist correction requires the planned hover height')
    angle=np.linalg.norm(cv2.Rodrigues(expected_pre[:3,:3].T@current[:3,:3])[0])
    if angle>np.deg2rad(3):raise ValueError('Wrist hover orientation differs from plan')
    diagnostics={}
    observed_obb=deepcopy(reference['target_obb'])
    tracking=reference.get('wrist_tracking')
    mask=None
    try:
        if tracking:
            from .wrist_tracking import track_translation
            delta=track_translation(image,calibration,current@np.asarray(calibration['T_tcp_camera']),
                                    observed_obb,tracking,diagnostics,max_distance=config['max_total_m'])
            if np.linalg.norm(delta)>config['max_total_m']:
                raise WristTargetError(0,'Tracked target outside correction bounds')
        else:
            if detector is None:
                from ultralytics import YOLO
                detector=YOLO(request['profile']['model_path'])
            detector.set_classes([reference['class_name']])
            from .wrist_detection import detect_translation
            delta,mask,_=detect_translation(detector,image,observed_obb,current,calibration,
                                           reference['class_name'],config['max_total_m'],diagnostics,directory,
                                           reference_area_fraction=reference.get('wrist_observation',{}).get('area_fraction'))
    finally:
        (directory/'wrist.json').write_text(json.dumps(diagnostics,allow_nan=False))
    if mask is not None:
        cv2.imwrite(str(directory/'wrist_mask.png'),mask*255)
        reference['wrist_observation']={'area_fraction':diagnostics['selected_area_fraction']}
    for key in ('T_pregrasp','T_grasp'):
        t=np.asarray(reference[key]);t[:3,3]+=delta;reference[key]=t.tolist()
    reference['target_obb']['center_m']=(np.asarray(reference['target_obb']['center_m'])+delta).tolist()
    total=np.asarray(reference['target_obb']['center_m'])-np.asarray(origin['target_obb']['center_m'])
    pre,converged,limited=bounded_hover(current,np.asarray(origin['T_pregrasp']),
        np.asarray(reference['T_pregrasp']),up,config)
    goal=np.asarray(reference['T_grasp']).copy();goal[:3,:3]=pre[:3,:3]
    if converged:
        # Convergence uses the actual visual estimate, not the clipped command.
        goal[:3,3]=pre[:3,3]+up*float((goal[:3,3]-pre[:3,3])@up)
    else:
        goal[:3,3]+=np.asarray(limited['limited_hover_offset_m'])-np.asarray(limited['requested_hover_offset_m'])
    planner=RightArmPlanner(request['urdf'])
    hover_path=[np.asarray(state['right'])] if converged else planner.cartesian_path(state['right'],pre)
    approach=planner.cartesian_path(hover_path[-1],goal)
    plan=deepcopy(reference)
    plan.update(start_joint_rad=state['right'],start_gripper_rad=state['right_gripper'],
                T_pregrasp=pre.tolist(),T_grasp=goal.tolist(),pregrasp_joint_rad=hover_path[-1].tolist(),
                pregrasp_joint_path=[q.tolist() for q in hover_path],approach_joint_rad=[q.tolist() for q in approach])
    scene={**request['profile']['scene'],'left_joint_rad':state['left'],'lift_joint_m':state['lift'],
           'left_finger_opening_m':float(np.clip(-state['left_gripper']/4.5,0,1))*.05}
    report,_,_=validate_plan(plan,request['urdf'],scene)
    if not tracking and not converged and report['status']=='passed':
        from .wrist_tracking import create_anchor
        reference['wrist_tracking']=create_anchor(image,mask,calibration,
            current@np.asarray(calibration['T_tcp_camera']),observed_obb,delta,directory/'tracking_anchor.npz')
    if tracking and not converged and report['status']=='passed' and not diagnostics.get('keyframe_recovery'):
        # Refresh only after direct original-anchor registration AND full plan
        # validation. A recovered keyframe cannot seed another recovery chain.
        from .wrist_keyframe import create_keyframe
        try:
            keyframe=create_keyframe(image,calibration,current@np.asarray(calibration['T_tcp_camera']),
                observed_obb,delta,tracking,directory/'tracking_keyframe.npz')
        except (ValueError,cv2.error) as exc:
            diagnostics['keyframe_refresh_error']=str(exc)
        else:
            reference['wrist_tracking']['keyframe']=keyframe
    diagnostics.update(delta_m=delta.tolist(),total_target_offset_m=total.tolist(),
                       converged=converged,**limited)
    (directory/'wrist.json').write_text(json.dumps(diagnostics,allow_nan=False))
    return dict(plan=plan,validation=report,reference_plan=reference,converged=converged)
