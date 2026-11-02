"""Geometry-bounded wrist detector recovery; confidence and correction limits unchanged."""
from itertools import product
from types import SimpleNamespace

from .compute_device import inference_device
import cv2
import numpy as np
from .common_utils import tensor_to_numpy


def search_roi(obb,base_camera,calibration,max_distance):
    center,size,rotation=(np.asarray(obb[k],float) for k in ('center_m','size_m','rotation'))
    corners=np.array(list(product((-1.,1.),repeat=3)))*size/2
    corners=corners@rotation.T+center
    offsets=np.array(list(product((-max_distance,max_distance),repeat=2)))@rotation[:,:2].T
    points=(corners[None]+offsets[:,None]).reshape(-1,3)
    camera=(points-base_camera[:3,3])@base_camera[:3,:3]
    if np.any(camera[:,2]<=.02):return None
    pixels=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),np.asarray(calibration['k']).reshape(3,3),
                             np.asarray(calibration['d']) if calibration['d'] else None)[0].reshape(-1,2)
    width,height=calibration['width'],calibration['height']
    low=np.maximum(np.floor(pixels.min(axis=0)-8),[0,0]).astype(int)
    high=np.minimum(np.ceil(pixels.max(axis=0)+9),[width,height]).astype(int)
    # Preserve surrounding context: excessively tight crops often split different faces of one object.
    extent=np.minimum(np.maximum(high-low,min(width,height)*2/3),[width,height]).astype(int)
    low=np.clip(np.floor((low+high-extent)/2),[0,0],np.array([width,height])-extent).astype(int)
    high=low+extent
    if min(high-low)<32 or np.prod(high-low)>.9*width*height:return None
    return (*low.tolist(),*high.tolist())


def mapped_instances(results,target,shape,roi=None):
    """Map original-resolution ROI masks back to the calibrated image, collapse strong duplicates."""
    height,width=shape;groups=[];raw=0;clipped=0
    x0,y0,x1,y1=roi or (0,0,width,height)
    for result in results:
        if result.boxes is None or result.masks is None:continue
        classes=tensor_to_numpy(result.boxes.cls).ravel();scores=tensor_to_numpy(result.boxes.conf).ravel()
        masks=tensor_to_numpy(result.masks.data)
        for index,(category,score) in enumerate(zip(classes,scores)):
            if result.names[int(category)]!=target or not np.isfinite(score) or score<.25 or index>=len(masks):continue
            raw+=1;mask=masks[index]>.5
            if mask.shape!=(y1-y0,x1-x0):raise ValueError('Wrist ROI masks must match original crop resolution')
            components,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
            if sum(stats[1:,cv2.CC_STAT_AREA]>=30)>64:raise ValueError('Too many fragmented wrist mask components')
            for component in range(1,components):
                if stats[component,cv2.CC_STAT_AREA]<30:continue
                part=labels==component;y,x=np.nonzero(part)
                if roi and (x.min()<3 or y.min()<3 or x.max()>=x1-x0-3 or y.max()>=y1-y0-3):
                    clipped+=1;continue
                full=np.zeros((height,width),bool);full[y0:y1,x0:x1]=part
                # Separate fragments are evaluated as independent hypotheses by the same geometry gates.
                # Merge only strongly overlapping/nested duplicates; do not merge separate objects.
                for group in groups:
                    intersection=np.count_nonzero(full&group[0]);area_a=np.count_nonzero(full);area_b=np.count_nonzero(group[0])
                    overlap=intersection/np.count_nonzero(full|group[0])
                    a=np.argwhere(full).mean(axis=0);b=np.argwhere(group[0]).mean(axis=0)
                    nested=(intersection/min(area_a,area_b)>.95 and min(area_a,area_b)/max(area_a,area_b)>.5 and
                            np.linalg.norm(a-b)<.1*np.sqrt(max(area_a,area_b)))
                    if overlap>.7 or nested:
                        group[0]|=full;group[1]=max(group[1],float(score));break
                else:
                    groups.append([full,float(score)])
                    if len(groups)>64:raise ValueError('Too many wrist detection hypotheses')
    merged=SimpleNamespace(names={0:target},boxes=SimpleNamespace(cls=np.zeros(len(groups)),
        conf=np.array([g[1] for g in groups])),masks=SimpleNamespace(data=np.array([g[0] for g in groups])))
    return [merged],dict(raw_masks=raw,unique_masks=len(groups),crop_clipped_components=clipped)


def detect_translation(detector,image,obb,base_tcp,calibration,target,max_distance,diagnostics,directory,*,reference_area_fraction=None):
    from .wrist import estimate_translation,WristTargetError
    diagnostics['compute_device']=inference_device()
    attempts=[];roi=None;last_error=WristTargetError(0)
    for view in ('full','roi'):
        if view=='roi':
            roi=search_roi(obb,base_tcp@np.asarray(calibration['T_tcp_camera']),calibration,max_distance)
            if roi is None:break
        crop=image if roi is None else image[roi[1]:roi[3],roi[0]:roi[2]]
        if roi:cv2.imwrite(str(directory/'wrist_roi.jpg'),crop)
        results=detector.predict(crop,verbose=False,device=inference_device(),imgsz=1280,conf=.25,iou=.45,retina_masks=True)
        mapped,summary=mapped_instances(results,target,image.shape[:2],roi)
        report=dict(view=view,roi_xyxy=roi,confidence_threshold=.25,**summary)
        attempts.append(report)
        try:
            output=estimate_translation(obb,base_tcp,calibration,mapped,target,max_distance,report,reference_area_fraction=reference_area_fraction)
            report['status']='passed'
            return output
        except WristTargetError as error:
            last_error=error
            report['status']='failed';report['error']=str(error)
            if error.count!=0:raise  # An ambiguous scene is never made unambiguous by cropping.
        finally:
            diagnostics.pop('error',None)
            diagnostics.update(report);diagnostics['attempts']=attempts
    raise last_error
