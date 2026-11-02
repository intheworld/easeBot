"""Read-only wrist evidence for current-session payload geometry; no motion."""
import json
from pathlib import Path
import cv2
import numpy as np


def observe_payload(request,detector=None):
    from .wrist import load_calibration,WristTargetError
    from .wrist_detection import detect_translation
    from .kinematics import RightArmFK
    config=request['profile']['wrist_refinement']
    calibration=load_calibration(config['calibration_file'])
    directory=Path(request['directory']);image=cv2.imread(str(directory/'rgb.jpg'))
    if image is None or image.shape[:2]!=(calibration['height'],calibration['width']):
        raise ValueError('Wrist image does not match calibration')
    payload=request['payload'];target=payload.get('detection_prompt','')
    if not target:return {'wrist_state':'no_target_model','object_visible':False}
    tcp=RightArmFK(request['urdf'])(request['state']['right'])
    obj=tcp@np.asarray(payload['T_tcp_object'])
    obb=dict(center_m=obj[:3,3].tolist(),rotation=obj[:3,:3].tolist(),size_m=payload['size_m'])
    if detector is None:
        from ultralytics import YOLO
        detector=YOLO(request['profile']['model_path'])
    detector.set_classes([target]);diagnostics={}
    try:
        _,mask,_=detect_translation(detector,image,obb,tcp,calibration,target,
            config['max_total_m'],diagnostics,directory)
        cv2.imwrite(str(directory/'holding_mask.png'),mask.astype(np.uint8)*255)
        result={'wrist_state':'target_consistent_with_payload','object_visible':True}
    except (WristTargetError,ValueError) as error:
        # Occlusion, out-of-view geometry, or ambiguity cannot prove an empty jaw.
        diagnostics['error']=str(error)
        result={'wrist_state':'inconclusive','object_visible':False}
    finally:
        (directory/'holding.json').write_text(json.dumps(diagnostics,allow_nan=False))
    return result
