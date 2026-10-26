"""Generic right-arm planning, preserving the gripper throughout the trajectory."""
import hashlib
import json

import cv2
import numpy as np

from ..trajectory import JointTrajectory
from ..grasp.planner import RightArmPlanner
from ..grasp.transforms import validate_transforms
from ..grasp.validation import (CollisionScreen, build_scene,
                                normalize_gripper_feedback, relative_tcp, right_arm_joint_limits, set_pose)
from .geometry import base_from_frame, pose_fields, target_transform


def trajectory_samples(start, waypoints, speed, max_step=.005):
    for elapsed, segment, q in JointTrajectory(start, waypoints, speed).samples(max_step):
        yield elapsed, 'start' if segment < 0 else f'segment_{segment}', q


def make_plan(request):
    args, state, profile = request['arguments'], request['state'], request['profile']
    if args['side'] != 'right' or args['path_mode'] not in ('joint','linear'):
        raise ValueError('Only right arm and joint/linear paths are supported')
    planner = RightArmPlanner(request['urdf'])
    start = np.asarray(state['right'], dtype=float)
    current = planner.fk(start)
    frame = base_from_frame(request['urdf'],state,args['frame_id'])
    goal_in_frame = target_transform(args, np.linalg.inv(frame) @ current)
    goal = frame @ goal_in_frame
    validate_transforms(goal[None], 'arm target')
    # Holding the current pose is a valid smoke test, including starts outside
    # the planner's extra IK margin but inside actual joint limits.
    if np.allclose(current,goal,atol=1e-8):
        waypoints = [start.copy()]
    elif args['path_mode'] == 'joint':
        waypoints = [planner.solve(goal,start)]
    else:
        waypoints = planner.cartesian_path(start,goal)
    duration = JointTrajectory(start,waypoints,profile['joint_speed_rad_s']).duration
    return {'schema':'hei.arm_plan/v1', 'side':'right', 'base_frame':'a_right_base_link',
            'tool_frame':'a_right_tcp', 'urdf_sha256':planner.fk.sha256,
            'target_frame':args['frame_id'], 'target_pose':pose_fields(goal_in_frame,args['frame_id']),
            'T_start':current.tolist(), 'T_goal':goal.tolist(), 'start_joint_rad':start.tolist(),
            'start_gripper_rad':state['right_gripper'], 'waypoints_rad':[q.tolist() for q in waypoints],
            'joint_speed_rad_s':profile['joint_speed_rad_s'], 'path_mode':args['path_mode'],
            'nominal_duration_sec':duration}


def validate_arm_plan(plan, urdf, scene):
    fk = RightArmPlanner(urdf).fk
    if plan['schema'] != 'hei.arm_plan/v1' or plan['urdf_sha256'] != fk.sha256:
        raise ValueError('Arm plan schema/URDF mismatch')
    if plan['base_frame'] != fk.base_frame or plan['tool_frame'] != fk.tool_frame:
        raise ValueError('Arm plan coordinate frame mismatch')
    grip = normalize_gripper_feedback(plan['start_gripper_rad'])
    lower, upper = right_arm_joint_limits(fk)
    model, data = build_scene(urdf,plan,scene,targets=[('start',plan['T_start'],[0.,.5,1.,1.]),
                                                     ('target',plan['T_goal'],[0.,1.,0.,1.])])
    screen = CollisionScreen(model,data,scene)
    missing = [k for k in ('table_top_z_m','left_joint_rad','lift_joint_m','left_finger_opening_m') if scene.get(k) is None]
    if scene.get('scene_confirmed') is not True:
        missing.append('scene_confirmed')
    problems, samples, max_error = set(), 0, 0.
    if not plan['waypoints_rad']:
        raise ValueError('Arm trajectory is empty')
    for elapsed, phase, q in trajectory_samples(plan['start_joint_rad'],plan['waypoints_rad'],plan['joint_speed_rad_s']):
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Invalid arm waypoint')
        if np.any(q < lower-1e-6) or np.any(q > upper+1e-6):
            problems.add('Arm trajectory violates joint limits')
        set_pose(model,data,q,grip)
        max_error = max(max_error,float(np.max(np.abs(relative_tcp(model,data)-fk(q)))))
        screen.check(phase,elapsed)
        samples += 1
    target = validate_transforms(np.asarray(plan['T_goal'])[None],'arm target')[0]
    actual = fk(plan['waypoints_rad'][-1])
    position_error = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
    rotation_error = float(np.linalg.norm(cv2.Rodrigues(target[:3,:3].T@actual[:3,:3])[0]))
    if position_error > .005 or rotation_error > np.deg2rad(3):
        problems.add('Arm target error exceeds 5 mm / 3 degrees')
    if max_error > 1e-5:
        problems.add('MuJoCo TCP differs from URDF FK')
    if plan['nominal_duration_sec'] > 295:
        problems.add('Arm trajectory exceeds the 300 second execution budget')
    return {'status':'failed' if screen.collisions or problems else 'incomplete' if missing else 'passed',
            'samples':samples, 'collision_pairs':list(screen.collisions.values()),
            'ignored_collision_pairs':list(screen.ignored.values()), 'errors':sorted(problems),
            'missing_scene_fields':missing, 'maximum_fk_matrix_error':max_error,
            'endpoint_position_error_m':position_error,'endpoint_angle_error_deg':float(np.degrees(rotation_error)),
            'scene':scene, 'urdf_sha256':fk.sha256,
            'plan_sha256':hashlib.sha256(json.dumps(plan,sort_keys=True).encode()).hexdigest(),
            'limitations':['Sampled kinematic collisions; no dynamics or continuous collision detection.',
                           'No carried-object or unmodeled-obstacle geometry.',
                           'Gripper held at measured angle with the existing linear finger model.']}


def compute(request):
    plan = make_plan(request)
    state = request['state']
    scene = {**request['profile']['scene'], 'left_joint_rad':state['left'], 'lift_joint_m':state['lift'],
             'left_finger_opening_m':float(np.clip(-state['left_gripper']/4.5,0,1))*.05}
    return {'plan':plan,'validation':validate_arm_plan(plan,request['urdf'],scene)}
