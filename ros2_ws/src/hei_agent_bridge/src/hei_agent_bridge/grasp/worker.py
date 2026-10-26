"""Offline compute subprocess: RGB-D files in, validated plan out. No ROS/serial/camera."""
import json
from pathlib import Path
import sys

# Run with the detector's own Python environment, independent of ROS's NumPy ABI.
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np
from hei_agent_bridge.grasp.common_utils import grasp_target
from hei_agent_bridge.grasp.planner import RightArmPlanner
from hei_agent_bridge.grasp.transforms import validate_transforms
from hei_agent_bridge.grasp.validation import validate_plan, normalize_gripper_feedback


def camera_transform(path):
    with np.load(path, allow_pickle=False) as data:
        for key, expected in [('mode','eye_to_hand'),('arm','right'),('base_frame','a_right_base_link')]:
            if str(data[key].reshape(-1)[0]) != expected:
                raise ValueError('Wrong eye-to-hand calibration metadata')
        return validate_transforms(data['T_result'][None], 'camera to right base')[0]


def object_plan(request):
    from hei_agent_bridge.grasp.obb import gripper_geometry, top_down_candidates, support_geometry
    profile,state = request['profile'],request['state']
    # A bad shared starting state cannot be repaired by another grasp candidate.
    # Check before IK rather than hiding the same validation error eight times.
    try:
        normalize_gripper_feedback(state['right_gripper'])
    except ValueError as error:
        raise ValueError(f'Invalid starting right gripper: {error}') from error
    obb = request['object']
    planner = RightArmPlanner(request['urdf'])
    geometry = gripper_geometry(request['urdf'],profile)
    up,support = support_geometry(request['urdf'],state,profile['scene'])
    if not np.allclose(up,np.asarray(obb['rotation'])[:,2],atol=1e-5):
        raise ValueError('OBB gravity frame differs from current model')
    candidates = top_down_candidates(obb,geometry,profile,support)
    directory = Path(request['directory'])
    (directory/'target.json').write_text(json.dumps({'strategy':'top_down_direct_ik','target_obb':obb,
        'gripper_geometry':geometry,'candidates':candidates},allow_nan=False))
    attempts, first_failed = [], None
    for index,candidate in enumerate(candidates):
        try:
            pre,goal = np.asarray(candidate['T_pregrasp']),np.asarray(candidate['T_grasp'])
            pre_q = planner.solve(pre,np.asarray(state['right']))
            approach = planner.cartesian_path(pre_q,goal)
            plan = dict(schema_version=3,strategy='top_down_direct_ik',
                        class_name=obb['target'],confidence=obb['confidence'],
                        base_frame=planner.fk.base_frame,tool_frame=planner.fk.tool_frame,
                        urdf_sha256=planner.fk.sha256,start_joint_rad=state['right'],
                        start_gripper_rad=state['right_gripper'],
                        motion_config={k:profile[k] for k in ('joint_speed_rad_s','gripper_open_rad','gripper_closed_rad')},
                        T_pregrasp=pre.tolist(),T_grasp=goal.tolist(),pregrasp_joint_rad=pre_q.tolist(),
                        approach_joint_rad=[q.tolist() for q in approach],
                        target_obb=obb,gripper_geometry=geometry,candidate_index=index,
                        jaw_width_m=candidate['jaw_width_m'])
            scene = {**profile['scene'],'left_joint_rad':state['left'],'lift_joint_m':state['lift'],
                     'left_finger_opening_m':float(np.clip(-state['left_gripper']/4.5,0,1))*.05}
            report,_,_ = validate_plan(plan,request['urdf'],scene)
            result = {'plan':plan,'validation':report}
            attempts.append({'candidate_index':index,'status':report['status'],
                             'errors':report['errors'],'collision_pairs':report['collision_pairs']})
            if first_failed is None:first_failed=result
            if report['status']=='passed':
                result['candidate_attempts']=attempts
                return result
        except ValueError as error:
            attempts.append({'candidate_index':index,'status':'failed','error':str(error)})
        finally:
            (directory/'candidates.json').write_text(json.dumps(attempts,allow_nan=False))
    if first_failed is not None:
        first_failed['candidate_attempts']=attempts
        return first_failed
    reason = attempts[0]['error'] if attempts else 'No candidates generated'
    raise ValueError(f'No reachable top-down grasp candidate: {reason}; inspect candidates.json')


def compute(request, detector=None):
    operation = request.get('operation')
    if operation == 'plan_object':
        return object_plan(request)
    if operation != 'perceive':
        raise ValueError(f'Unsupported grasp operation: {operation!r}')
    target = grasp_target(request['target'])
    profile = request['profile']
    directory = Path(request['directory'])
    image = cv2.imread(str(directory / 'rgb.jpg'))
    depth = np.load(directory / 'depth.npy', allow_pickle=False)
    info = request['info']
    if image is None or image.shape[:2] != depth.shape or not np.isfinite(info['k']).all():
        raise ValueError('Invalid RGB-D snapshot')
    if info['distortion_model'] not in ('plumb_bob', 'rational_polynomial', '') or any(abs(v) > 1e-8 for v in info['d']):
        raise ValueError('Grasp estimator requires undistorted RGB-D calibration')
    if detector is None:
        from ultralytics import YOLO
        detector = YOLO(profile['model_path'])
    detector.set_classes([target])
    # Small tabletop objects need a larger detector input; depth projection still
    # uses the original aligned RGB-D resolution and intrinsics.
    inference_size = 1280
    results = detector.predict(image, verbose=False, device='cpu', conf=.25, iou=.45,
                               imgsz=inference_size)
    from hei_agent_bridge.grasp.obb import segmented_cloud, visible_obb, support_geometry
    transform = camera_transform(request['calibration'])
    diagnostics = {'confidence_threshold': .25, 'inference_size': inference_size,
                   'image_width': image.shape[1],
                   'image_height': image.shape[0]}
    try:
        confidence,points,mask = segmented_cloud(results,depth,info['k'],target,transform,diagnostics)
    finally:
        (directory/'perception.json').write_text(json.dumps(diagnostics,allow_nan=False))
    up,_ = support_geometry(request['urdf'],request['state'],profile['scene'])
    obb = visible_obb(points,up)
    obb.update(target=target,confidence=confidence)
    np.save(directory/'object_cloud.npy',points,allow_pickle=False)
    cv2.imwrite(str(directory/'object_mask.png'),mask.astype(np.uint8)*255)
    return {'object':obb}


def main():
    directory = Path(sys.argv[1])
    try:
        result = compute(json.loads((directory / 'request.json').read_text()))
    except Exception as exc:
        result = {'error': f'{type(exc).__name__}: {exc}'}
    output = directory / 'result.json'
    temporary = output.with_suffix('.part')
    temporary.write_text(json.dumps(result, allow_nan=False), encoding='utf-8')
    temporary.replace(output)
    return 1 if 'error' in result else 0


if __name__ == '__main__':
    raise SystemExit(main())
