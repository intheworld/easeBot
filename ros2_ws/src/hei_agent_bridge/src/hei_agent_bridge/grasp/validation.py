"""Trajectory sampling and collision policy, independent of offline scene building."""
from __future__ import annotations

import hashlib
import json
import math

import cv2
import numpy as np

from ..trajectory import JointTrajectory
from .kinematics import RightArmFK
from .transforms import validate_transforms


def right_arm_joint_limits(fk):
    """Execution policy: joint 2/3 no longer have a zero-radian upper bound."""
    limits = [j.find('limit') for j in fk.chain if j.get('type') == 'revolute']
    lower = np.maximum([float(x.get('lower')) for x in limits],
                       [-.3, -3.14, -3.14, -1.4, -1.57, -3.14])
    upper = np.minimum([float(x.get('upper')) for x in limits],
                       [1.5, 0., 0., 1.57, 1.57, 3.14])
    upper[1:3] = np.inf  # User explicitly removed these software upper bounds.
    return lower, upper


def normalize_gripper_feedback(value):
    """Allow 0.01 rad encoder zero noise; command/model limits remain -4.5..0."""
    value = float(value)
    if not np.isfinite(value) or not -4.51 <= value <= .01:
        raise ValueError(f'Gripper feedback {value!r} rad outside -4.5..0 rad (0.01 rad tolerance)')
    return float(np.clip(value, -4.5, 0.))


def motion_blend(t):
    return t*t*t*(10+t*(-15+6*t))


def _vector(value, count, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (count,) or not np.isfinite(value).all():
        raise ValueError(f'{name}: expected {count} finite numbers')
    return value


def trajectory_samples(plan, max_step=.005):
    """Sample the OBB grasp: initial hold, open, pregrasp, approach, close."""
    if not np.isfinite(max_step) or not 0 < max_step <= .02:
        raise ValueError('Invalid sampling step')
    cfg = plan['motion_config']
    speed = float(cfg['joint_speed_rad_s'])
    q = _vector(plan['start_joint_rad'], 6, 'start')
    grip = normalize_gripper_feedback(plan['start_gripper_rad'])
    opened, closed = float(cfg['gripper_open_rad']), float(cfg['gripper_closed_rad'])
    if not np.isfinite([opened, closed]).all() or not -4.5 <= opened < closed <= 0:
        raise ValueError(f'Invalid gripper targets: open={opened}, closed={closed}; expected -4.5 <= open < closed <= 0')
    approach = [_vector(x, 6, 'approach') for x in plan['approach_joint_rad']]
    pregrasp = [_vector(x, 6, 'pregrasp') for x in plan.get('pregrasp_joint_path',[plan['pregrasp_joint_rad']])]
    if not pregrasp or not np.allclose(pregrasp[-1],plan['pregrasp_joint_rad'],atol=1e-8):
        raise ValueError('Pregrasp path endpoint differs from pregrasp joints')
    if not approach:
        raise ValueError('Approach must be nonempty')
    phases = [('open', [q.copy()], opened),
              ('pregrasp', pregrasp, opened),
              ('approach', approach, opened), ('close', [approach[-1]], closed)]
    elapsed = 0.
    yield elapsed, 'start', q.copy(), grip
    for phase, path, next_grip in phases:
        if phase in ('open', 'close'):
            count = max(1, int(math.ceil(1.875*abs(next_grip-grip)/90./max_step)))
            for fraction in np.linspace(0, 1, count+1)[1:]:
                yield elapsed+5.*fraction, phase, q.copy(), grip+(next_grip-grip)*motion_blend(float(fraction))
            elapsed += 5.
        else:
            trajectory = JointTrajectory(q, path, speed)
            for t, _, position in trajectory.samples(max_step):
                yield elapsed+t, phase, position, grip
            elapsed += trajectory.duration
        q, grip = path[-1].copy(), next_grip


# Compatibility entry points for saved offline replay scripts. Online callers
# use collision.build_scene; MuJoCo is only reached through explicit fallback.
from ..collision.meshes import stl_components


def build_scene(urdf, plan, scene, *, targets=None):
    from ..collision.mujoco_scene import build_scene as build_offline_scene
    return build_offline_scene(urdf,plan,scene,targets=targets)


def set_pose(model, data, q, grip):
    if getattr(model,'backend',None)=='fcl':
        model.set_pose(q,grip)
    else:
        from ..collision.mujoco_scene import set_pose as set_offline_pose
        set_offline_pose(model,data,q,grip)


def relative_tcp(model,data):
    if getattr(model,'backend',None)=='fcl':return model.tcp()
    from ..collision.mujoco_scene import relative_tcp as offline_tcp
    return offline_tcp(model,data)


class CollisionScreen:
    """Shared sampled collision policy for grasp and generic arm trajectories."""
    def __init__(self, model, data, scene):
        self.model, self.data = model, data
        self.collisions, self.ignored = {}, {}
        self.exclusions = {}
        for exclusion in scene.get('collision_exclusions', []):
            bodies = exclusion.get('bodies', [])
            if len(bodies) != 2 or not all(isinstance(b, str) for b in bodies) or bodies[0] == bodies[1]:
                raise ValueError('Collision exclusion must name two distinct bodies')
            for body in bodies:
                model.body(body)  # Reject misspelled names rather than silently ignoring them.
            depth = float(exclusion['max_penetration_m'])
            if not np.isfinite(depth) or depth < 0 or not exclusion.get('reason'):
                raise ValueError('Collision exclusion requires a finite nonnegative depth and reason')
            key = tuple(sorted(bodies))
            if key in self.exclusions:
                raise ValueError('Duplicate collision exclusion')
            self.exclusions[key] = dict(max_penetration_m=depth, reason=exclusion['reason'])
        self.clearance = float(scene.get('clearance_m', .003))
        self.table_gripper_clearance = float(scene.get('table_gripper_clearance_m', 0.))
        if not np.isfinite(self.table_gripper_clearance) or not -.003 <= self.table_gripper_clearance <= 0:
            raise ValueError('Table/gripper clearance must be between -0.003 and 0 m')
        self.support_contact=None;self.support_separated=False
        self.payload_padding=float(scene.get('payload_padding_m',.003))
        self.release_body = scene.get('release_object_body','')
        self.right_fingers = {'a_right_end_link_L_finger', 'a_right_end_link_R_finger'}

    def check(self, phase, elapsed):
        support_seen=False
        for contact in self.data.contact[:self.data.ncon]:
            g1, g2 = int(contact.geom1), int(contact.geom2)
            b1, b2 = self.model.body(self.model.geom_bodyid[g1]).name, self.model.body(self.model.geom_bodyid[g2]).name
            if not (b1.startswith('a_right') or b2.startswith('a_right') or
                    {b1, b2} & getattr(self.model, 'moving', set())):
                continue
            # Only the gripping fingers may contact the carried object. The
            # payload still collides with the arm, table, base and placed objects.
            if ({b1,b2} & self.right_fingers and
                    ('a_right_payload' in (b1,b2) or self.release_body in (b1,b2) and self.release_body)):
                continue
            # Expected finger-tip contact at fully closed jaws (CAD tolerance).
            if {b1, b2} == {'a_right_end_link_L_finger', 'a_right_end_link_R_finger'} and contact.dist >= -.001:
                continue
            region_floor=any(name.startswith('placed_') and name.endswith('_floor') for name in (b1,b2))
            if 'a_right_payload' in (b1,b2) and (region_floor or 'validation_table' in (self.model.geom(g1).name,self.model.geom(g2).name)):
                center=self.data.xpos[self.model.body('a_right_payload').id].copy()
                if self.support_contact is None and elapsed==0 and contact.dist>=-(2 if region_floor else 1)*self.payload_padding-1e-6:
                    self.support_contact=(center,float(contact.dist))
                if self.support_contact is not None and not self.support_separated:
                    origin,depth=self.support_contact
                    separating=(elapsed==0 or center[2]>origin[2]+1e-8 or np.linalg.norm(center-origin)<1e-8)
                    if separating and np.linalg.norm(center[:2]-origin[:2])<=.001 and center[2]>=origin[2]-1e-5 and contact.dist>=depth-1e-5:
                        support_seen=True
                        continue
            if contact.dist < self.clearance:
                key = tuple(sorted((g1, g2)))
                item = dict(phase=phase, time_s=float(elapsed), distance_m=float(contact.dist),
                            bodies=[b1, b2], geom_ids=[g1, g2], geoms=[self.model.geom(g1).name or f'geom_{g1}',
                                                  self.model.geom(g2).name or f'geom_{g2}'])
                item['constraint_kind'] = ('unobserved_space' if any('_coverage_unknown_' in b for b in (b1,b2))
                                           else 'observed_geometry')
                exclusion = self.exclusions.get(tuple(sorted((b1, b2))))
                destination = self.collisions
                if exclusion and contact.dist >= -exclusion['max_penetration_m']:
                    item['exclusion_reason'] = exclusion['reason']
                    destination = self.ignored
                if ('validation_table' in item['geoms'] and {b1, b2} & self.right_fingers
                        and contact.dist >= self.table_gripper_clearance):
                    item['exclusion_reason'] = f'Table/right finger tolerance: {self.table_gripper_clearance:g} m'
                    destination = self.ignored
                if key not in destination or item['distance_m'] < destination[key]['distance_m']:
                    destination[key] = item

        if self.support_contact is not None and not support_seen:self.support_separated=True


def validate_plan(plan, urdf, scene, *, backend="fcl"):
    from ..collision import build_scene as build_online_scene
    if backend not in ("fcl", "mujoco"):
        raise ValueError("Unknown collision backend")
    if plan.get('schema_version') != 3 or plan.get('strategy') != 'top_down_direct_ik':
        raise ValueError('Unsupported grasp plan; regenerate an OBB plan')
    required = ('motion_config', 'start_gripper_rad', 'start_joint_rad', 'pregrasp_joint_rad',
                'approach_joint_rad', 'T_grasp', 'T_pregrasp')
    missing_plan = [key for key in required if key not in plan]
    if missing_plan:
        raise ValueError('Plan missing '+', '.join(missing_plan)+'; regenerate an OBB plan')
    fk = RightArmFK(urdf)
    joint_lower, joint_upper = right_arm_joint_limits(fk)
    if plan.get('urdf_sha256') != fk.sha256:
        raise ValueError('Plan/validation URDF hashes differ; regenerate the plan')
    if plan.get('base_frame') != fk.base_frame or plan.get('tool_frame') != fk.tool_frame:
        raise ValueError('Plan coordinate frames do not match the robot')
    scene = dict(scene)
    known=plan.get('target_obb',{}).get('known_obstacle_id')
    if known:scene['release_object_body']='placed_'+known
    if scene.get('left_joint_rad') is not None:
        scene['left_joint_rad'] = _vector(scene['left_joint_rad'], 6, 'left joints').tolist()
    for key in ('lift_joint_m', 'left_finger_opening_m'):
        if scene.get(key) is not None and not np.isfinite(float(scene[key])):
            raise ValueError(f'Invalid {key}')
    builder = build_online_scene if backend == "fcl" else build_scene
    model, data = builder(urdf, plan, scene)
    screen = CollisionScreen(model, data, scene)
    problems = []
    missing = [key for key in ('table_top_z_m', 'left_joint_rad', 'lift_joint_m', 'left_finger_opening_m')
               if scene.get(key) is None]
    if scene.get('scene_confirmed') is not True:
        missing.append('scene_confirmed')
    samples = 0
    max_fk_error = 0.
    from ..collision.cuda_batch import posed_samples
    for elapsed, phase, q, grip in posed_samples(model,data,trajectory_samples(plan)):
        samples += 1
        for i, value in enumerate(q):
            if not joint_lower[i]-1e-6 <= value <= joint_upper[i]+1e-6:
                problems.append(f'{phase}: right joint {i+1} violates hardware limits')
        error = float(np.max(np.abs(relative_tcp(model, data)-fk(q))))
        max_fk_error = max(max_fk_error, error)
        screen.check(phase, elapsed)
        if backend == 'fcl' and screen.collisions:
            break
    paths = [('pregrasp',plan.get('pregrasp_joint_path',[plan['pregrasp_joint_rad']])),('approach',plan['approach_joint_rad'])]
    for phase, waypoints in paths:
        for q in waypoints:
            for i, value in enumerate(q):
                low, high = joint_lower[i], joint_upper[i]
                if not low+.05-1e-6 <= value <= high-.05+1e-6:
                    problems.append(f'{phase}: planned joint {i+1} violates IK margin')
    if max_fk_error > 1e-5:
        problems.append('Collision backend TCP differs from calibrated URDF FK')
    endpoint_errors = {}
    endpoints = [('pregrasp',plan['pregrasp_joint_rad'],'T_pregrasp'),
                 ('grasp',plan['approach_joint_rad'][-1],'T_grasp')]
    for name, q, key in endpoints:
        set_pose(model, data, _vector(q, 6, name), float(plan['motion_config']['gripper_open_rad']))
        target = validate_transforms(np.asarray(plan[key])[None], key)[0]
        actual = relative_tcp(model, data)
        pos = float(np.linalg.norm(actual[:3, 3]-target[:3, 3]))
        angle = float(np.linalg.norm(cv2.Rodrigues(target[:3, :3].T @ actual[:3, :3])[0]))
        endpoint_errors[name] = dict(position_error_m=pos, angle_error_deg=float(np.degrees(angle)))
        if pos > .005 or angle > np.deg2rad(3):
            problems.append(f'{name}: TCP target error exceeds 5 mm / 3 degrees')
    status = 'failed' if screen.collisions or problems else ('incomplete' if missing else 'passed')
    result = dict(status=status, samples=samples, collision_backend=backend, collision_accelerator=getattr(model,"collision_accelerator","cpu"),
                  start_gripper_feedback_rad=float(plan['start_gripper_rad']),
                  start_gripper_model_rad=normalize_gripper_feedback(plan['start_gripper_rad']), collision_pairs=list(screen.collisions.values()),
                  ignored_collision_pairs=list(screen.ignored.values()),
                  errors=sorted(set(problems)), missing_scene_fields=missing,
                  endpoint_errors=endpoint_errors, maximum_fk_matrix_error=max_fk_error,
                  collision_geometry='convex hull per disconnected STL solid',
                  joint_limits_rad=dict(lower=joint_lower.tolist(),
                                        upper=[float(v) if np.isfinite(v) else None for v in joint_upper]),
                  scene=scene, urdf_sha256=fk.sha256,
                  plan_sha256=hashlib.sha256(json.dumps(plan, sort_keys=True).encode()).hexdigest(),
                  limitations=['Kinematic sampled screening; not continuous collision or dynamics validation.',
                               'Each disconnected CAD solid uses a convex hull; concavity inside a solid is still approximated.',
                               'Known carried/placed boxes are checked; unmodeled obstacles, hidden surfaces and real tracking/calibration errors remain unknown.',
                               'Finger opening uses a linear motor-to-URDF approximation.'])
    return result, model, data
