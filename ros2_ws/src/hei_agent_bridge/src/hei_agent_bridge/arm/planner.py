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
    from ..grasp.planner_backend import get_planner, cached_cartesian_path
    planner = (RightArmPlanner(request['urdf']) if args.get('joint_home') is True
               else get_planner(request['urdf'],profile.get('ik_backend','auto')))
    start = np.asarray(state['right'], dtype=float)
    current = planner.fk(start)
    frame = base_from_frame(request['urdf'],state,args['frame_id'])
    if args.get('joint_home') is True:
        # Home is an exact joint target, not IK for a TCP pose with potentially
        # different joint solutions. It never changes the motor encoder zeros.
        home=np.zeros(6)
        lower,upper=right_arm_joint_limits(planner.fk)
        if np.any(home<lower) or np.any(home>upper):
            raise ValueError('Zero joint pose lies outside URDF limits')
        waypoints=[home]
        goal=planner.fk(home)
        goal_in_frame=np.linalg.inv(frame) @ goal
    else:
        goals=[]
        if args.get('relative'):
            delta=np.array([args[k] for k in ('dx_m','dy_m','dz_m')],dtype=float)
            if not np.isfinite(delta).all():raise ValueError('Relative displacement must be finite')
            goal=current.copy()
            rotation=current[:3,:3] if args.get('tool_relative') else frame[:3,:3]
            goal[:3,3]+=rotation@delta
            goals=[goal]
        elif 'waypoints' in args:
            if not 1<=len(args['waypoints'])<=32:raise ValueError('Path needs 1..32 waypoints')
            goals=[frame@target_transform(item,np.linalg.inv(frame)@current) for item in args['waypoints']]
        else:
            goals=[frame@target_transform(args,np.linalg.inv(frame)@current)]
        waypoints=[];seed=start.copy()
        for goal_index,goal in enumerate(goals):
            validate_transforms(goal[None], 'arm target')
            if np.allclose(planner.fk(seed),goal,atol=1e-8):segment=[seed.copy()]
            elif args['path_mode']=='joint':segment=[planner.solve(goal,seed)]
            elif args.get('_placement_soft_intermediate'):
                segment=cached_cartesian_path(planner,seed,goal,strict_intermediate=False,
                    strict_endpoint=goal_index==len(goals)-1)
            else:segment=cached_cartesian_path(planner,seed,goal)
            waypoints.extend(segment);seed=waypoints[-1]
        goal_in_frame=np.linalg.inv(frame)@goal
    duration = JointTrajectory(start,waypoints,profile['joint_speed_rad_s']).duration
    return {'ik_backend':getattr(planner,'backend','scipy'), 'schema':'hei.arm_plan/v1', 'side':'right', 'base_frame':'a_right_base_link',
            'tool_frame':'a_right_tcp', 'urdf_sha256':planner.fk.sha256,
            'target_frame':args['frame_id'], 'target_pose':pose_fields(goal_in_frame,args['frame_id']),
            'T_start':current.tolist(), 'T_goal':goal.tolist(), 'start_joint_rad':start.tolist(),
            'start_gripper_rad':state['right_gripper'], 'waypoints_rad':[q.tolist() for q in waypoints],
            'joint_speed_rad_s':profile['joint_speed_rad_s'], 'path_mode':args['path_mode'],
            'nominal_duration_sec':duration,'carry':request.get('carry'),
            'waypoint_targets':[g.tolist() for g in goals] if not args.get('joint_home') else [goal.tolist()]}


def validate_arm_plan(plan, urdf, scene, *, backend="fcl", _collision_scene=None):
    from ..collision import build_scene as build_online_scene
    if backend not in ("fcl", "mujoco"):
        raise ValueError("Unknown collision backend")
    fk = RightArmPlanner(urdf).fk
    if plan['schema'] != 'hei.arm_plan/v1' or plan['urdf_sha256'] != fk.sha256:
        raise ValueError('Arm plan schema/URDF mismatch')
    if plan['base_frame'] != fk.base_frame or plan['tool_frame'] != fk.tool_frame:
        raise ValueError('Arm plan coordinate frame mismatch')
    grip = normalize_gripper_feedback(plan['start_gripper_rad'])
    lower, upper = right_arm_joint_limits(fk)
    builder = build_online_scene if backend == "fcl" else build_scene
    if _collision_scene is None:
        model, data = builder(urdf,plan,scene,targets=[('start',plan['T_start'],[0.,.5,1.,1.]),
                                                     ('target',plan['T_goal'],[0.,1.,0.,1.])])
    else:
        model, data = _collision_scene
        from ..collision.fcl_scene import context_signature
        if backend != 'fcl' or model.signature != context_signature(urdf,plan,scene):
            raise ValueError('Collision context differs from request geometry')
    screen = CollisionScreen(model,data,scene)
    missing = [k for k in ('table_top_z_m','left_joint_rad','lift_joint_m','left_finger_opening_m') if scene.get(k) is None]
    if scene.get('scene_confirmed') is not True:
        missing.append('scene_confirmed')
    problems, samples, max_error = set(), 0, 0.
    if not plan['waypoints_rad']:
        raise ValueError('Arm trajectory is empty')
    from ..collision.cuda_batch import posed_samples
    rows = ((t,p,q,grip) for t,p,q in trajectory_samples(plan['start_joint_rad'],plan['waypoints_rad'],plan['joint_speed_rad_s']))
    for elapsed, phase, q, _ in posed_samples(model,data,rows):
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Invalid arm waypoint')
        if np.any(q < lower-1e-6) or np.any(q > upper+1e-6):
            problems.add('Arm trajectory violates joint limits')
        max_error = max(max_error,float(np.max(np.abs(relative_tcp(model,data)-fk(q)))))
        screen.check(phase,elapsed)
        samples += 1
        if backend == "fcl" and screen.collisions:
            break
    if 'gripper_end_rad' in plan and not screen.collisions:
        endpoint=normalize_gripper_feedback(plan['gripper_end_rad'])
        q=np.asarray(plan['waypoints_rad'][-1])
        rows = ((float(i),'release',q,v) for i,v in enumerate(np.linspace(grip,endpoint,max(2,int(abs(endpoint-grip)/.02)+1))))
        for fraction,phase,_,value in posed_samples(model,data,rows):
            screen.check(phase,fraction)
            samples+=1
            if backend == "fcl" and screen.collisions:
                break
    target = validate_transforms(np.asarray(plan['T_goal'])[None],'arm target')[0]
    actual = fk(plan['waypoints_rad'][-1])
    position_error = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
    rotation_error = float(np.linalg.norm(cv2.Rodrigues(target[:3,:3].T@actual[:3,:3])[0]))
    if position_error > .005 or rotation_error > np.deg2rad(3):
        problems.add('Arm target error exceeds 5 mm / 3 degrees')
    if max_error > 1e-5:
        problems.add('Collision backend TCP differs from URDF FK')
    if plan['nominal_duration_sec'] > 295:
        problems.add('Arm trajectory exceeds the 300 second execution budget')
    return {'status':'failed' if screen.collisions or problems else 'incomplete' if missing else 'passed',
            'collision_backend':backend, 'collision_accelerator':getattr(model,'collision_accelerator','cpu'), 'stopped_on_collision':bool(screen.collisions) and backend=='fcl', 'samples':samples, 'collision_pairs':list(screen.collisions.values()),
            'ignored_collision_pairs':list(screen.ignored.values()), 'errors':sorted(problems),
            'missing_scene_fields':missing, 'maximum_fk_matrix_error':max_error,
            'endpoint_position_error_m':position_error,'endpoint_angle_error_deg':float(np.degrees(rotation_error)),
            'scene':scene, 'urdf_sha256':fk.sha256,
            'plan_sha256':hashlib.sha256(json.dumps(plan,sort_keys=True).encode()).hexdigest(),
            'limitations':['Sampled kinematic collisions; no dynamics or continuous collision detection.',
                           'Carried/placed OBB geometry is modeled; unobserved obstacles and hidden object surfaces remain unknown.',
                           'Gripper held at measured angle with the existing linear finger model.']}


def compute(request):
    state = request['state']
    placing=request['arguments'].get('operation')=='place'
    scene = {**request['profile']['scene'], 'left_joint_rad':state['left'], 'lift_joint_m':state['lift'],
             'left_finger_opening_m':float(np.clip(-state['left_gripper']/4.5,0,1))*.05}
    scene['payload_padding_m']=request['profile'].get('payload_padding_m',.003)
    if placing:
        from .place import make_place_plan,validate_place_plan,compute_region_place
        if 'region' in request['arguments']:return compute_region_place(request,scene)
        plan=make_place_plan(request)
    else:plan=make_plan(request)
    validate=validate_place_plan if placing else validate_arm_plan
    return {'plan':plan,'validation':validate(plan,request['urdf'],scene)}
