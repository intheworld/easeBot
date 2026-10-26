"""Offline, kinematic MuJoCo validation. Never imports a hardware controller."""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

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
    if not approach:
        raise ValueError('Approach must be nonempty')
    phases = [('open', [q.copy()], opened),
              ('pregrasp', [_vector(plan['pregrasp_joint_rad'], 6, 'pregrasp')], opened),
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


def stl_components(path):
    """Partition disconnected binary STL solids without excluding any geometry."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    raw = Path(path).read_bytes()
    if len(raw) < 84:
        raise ValueError('Unsupported STL')
    count = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84+50*count:
        raise ValueError('Expected binary STL')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    points = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    vertices, inverse = np.unique(points, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    a, b = faces.ravel(), faces[:, [1, 2, 0]].ravel()
    graph = coo_matrix((np.ones(len(a)), (a, b)), shape=(len(vertices), len(vertices)))
    number, labels = connected_components(graph, directed=False)
    pieces = [vertices[labels == i].astype(float) for i in range(number)]
    if any(len(v) < 4 or np.linalg.matrix_rank(v-v.mean(0)) < 3 for v in pieces):
        # Never discard degenerate fragments; keep the original collider instead.
        return [vertices.astype(float)]
    return pieces


def split_collision_meshes(spec, urdf):
    """Use one conservative convex hull per disconnected CAD solid, not per assembly."""
    import mujoco
    urdf = Path(urdf).resolve()
    compiler = ET.parse(urdf).getroot().find('mujoco/compiler')
    meshdir = compiler.get('meshdir', '') if compiler is not None else ''
    for geom in list(spec.geoms):
        if geom.group != 0 or not geom.meshname:
            continue
        asset = spec.mesh(geom.meshname)
        source = urdf.parent / meshdir / Path(asset.file).name
        if not source.is_file() or source.suffix.lower() != '.stl':
            continue
        try:
            pieces = stl_components(source)
        except ValueError:
            continue
        if len(pieces) == 1:
            continue
        body = geom.parent
        for index, vertices in enumerate(pieces):
            name = f'collision_{body.name}_part{index}'
            spec.add_mesh(name=name, uservert=vertices.ravel().tolist(), scale=asset.scale,
                          refpos=asset.refpos, refquat=asset.refquat)
            body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname=name,
                          pos=geom.pos, quat=geom.quat, group=0, contype=1, conaffinity=1,
                          rgba=[.5, .5, .5, .3])
        spec.delete(geom)


def build_scene(urdf, plan, scene, *, targets=None):
    import mujoco
    spec = mujoco.MjSpec.from_file(str(Path(urdf).resolve()))
    spec.worldbody.add_light(name='validation_light', pos=[2., -2., 3.],
                             dir=[-1., 1., -1.],
                             diffuse=[.8, .8, .8], ambient=[.3, .3, .3])
    spec.worldbody.add_geom(name='validation_ground', type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[3., 3., .1], rgba=[.18, .2, .23, 1.],
                            contype=1, conaffinity=1)
    split_collision_meshes(spec, urdf)
    # Anchor the measured table and targets to the calibrated right-arm base.
    base = spec.body('a_right_base_link')
    if base is None:
        raise ValueError('MuJoCo model missing a_right_base_link')
    if scene.get('table_top_z_m') is not None:
        z = float(scene['table_top_z_m'])
        xy = _vector(scene.get('table_center_xy_m', [.92, 0]), 2, 'table center')
        size = _vector(scene.get('table_size_xy_m', [.96, 1.24]), 2, 'table size')
        thickness = float(scene.get('table_thickness_m', .04))
        if not np.isfinite([z, thickness]).all() or thickness <= 0 or np.any(size <= 0):
            raise ValueError('Invalid table dimensions')
        table_frame = scene.get('table_frame', 'a_right_base_link')
        if table_frame not in ('a_right_base_link', 'base_footprint'):
            raise ValueError('table_frame must be a_right_base_link or base_footprint')
        table_parent = spec.body(table_frame)
        if table_parent is None:
            raise ValueError(f'Missing table coordinate frame: {table_frame}')
        table_parent.add_geom(name='validation_table', type=mujoco.mjtGeom.mjGEOM_BOX,
                      pos=[*xy, z-thickness/2], size=[* (size/2), thickness/2],
                      rgba=[.45, .32, .18, .5], contype=1, conaffinity=1)
    # Mark the planned grasp/approach, not an invented knife collision mesh.
    if targets is None:
        targets = [('grasp', plan['T_grasp'], [1., .8, 0., 1.]),
                   ('pregrasp', plan['T_pregrasp'], [0., 1., 0., 1.])]
    for name, value, color in targets:
        target = validate_transforms(np.asarray(value, dtype=float)[None], name)[0]
        base.add_site(name=f'validation_{name}', pos=target[:3, 3], size=[.008], rgba=color)
        for axis in range(3):
            endpoint = target[:3, 3]+target[:3, axis]*.06
            rgba = [0., 0., 0., 1.]; rgba[axis] = 1.
            base.add_site(name=f'validation_{name}_axis{axis}', type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                          fromto=np.r_[target[:3, 3], endpoint], size=[.002], rgba=rgba)
    model = spec.compile()
    # Remove the same upper-bound restriction from the kinematic preview.
    # Lower bounds remain checked explicitly by validate_plan.
    for name in ('a_right_joint2', 'a_right_joint3'):
        model.jnt_limited[model.joint(name).id] = 0
    data = mujoco.MjData(model)
    for i, val in enumerate(scene.get('left_joint_rad') or [0., -.5, -.5, 0., 0., 0.]):
        data.qpos[model.joint(f'b_left_joint{i+1}').qposadr] = val
    data.qpos[model.joint('lift_joint').qposadr] = float(scene.get('lift_joint_m') or 0.)
    left_open = float(scene.get('left_finger_opening_m') or 0.)
    for side, sign in [('L', 1), ('R', -1)]:
        data.qpos[model.joint(f'b_left_end_joint_{side}_finger').qposadr] = sign*left_open
    clearance = float(scene.get('clearance_m', .003))
    if not np.isfinite(clearance) or not 0 <= clearance <= .03:
        raise ValueError('Invalid collision clearance')
    # URDF group 0 is collision geometry, group 1 is visual geometry.
    for i in range(model.ngeom):
        if model.geom_group[i] == 0:
            model.geom_contype[i] = 1
            model.geom_conaffinity[i] = 1
            model.geom_margin[i] = clearance
    model.opt.gravity[:] = 0
    mujoco.mj_forward(model, data)
    for i in range(model.njnt):
        if model.jnt_limited[i]:
            value = data.qpos[model.jnt_qposadr[i]]
            low, high = model.jnt_range[i]
            if not low-1e-6 <= value <= high+1e-6:
                raise ValueError(f'Scene joint outside limits: {model.joint(i).name}')
    return model, data


def set_pose(model, data, q, grip):
    import mujoco
    for i, value in enumerate(q):
        data.qpos[model.joint(f'a_right_joint{i+1}').qposadr] = value
    # HEI follower convention: -4.5 rad open, 0 rad closed; URDF slide is 0..0.05 m.
    opening = float(np.clip(-grip/4.5, 0., 1.))*.05
    for side, sign in [('L', 1), ('R', -1)]:
        data.qpos[model.joint(f'a_right_end_joint_{side}_finger').qposadr] = sign*opening
    mujoco.mj_forward(model, data)


def relative_tcp(model, data):
    base, tcp = model.body('a_right_base_link').id, model.body('a_right_tcp').id
    rotation = data.xmat[base].reshape(3, 3)
    target = np.eye(4)
    target[:3, :3] = rotation.T @ data.xmat[tcp].reshape(3, 3)
    target[:3, 3] = rotation.T @ (data.xpos[tcp]-data.xpos[base])
    return target


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
        self.right_fingers = {'a_right_end_link_L_finger', 'a_right_end_link_R_finger'}

    def check(self, phase, elapsed):
        for contact in self.data.contact[:self.data.ncon]:
            g1, g2 = int(contact.geom1), int(contact.geom2)
            b1, b2 = self.model.body(self.model.geom_bodyid[g1]).name, self.model.body(self.model.geom_bodyid[g2]).name
            if not (b1.startswith('a_right') or b2.startswith('a_right')):
                continue
            # Expected finger-tip contact at fully closed jaws (CAD tolerance).
            if {b1, b2} == {'a_right_end_link_L_finger', 'a_right_end_link_R_finger'} and contact.dist >= -.001:
                continue
            if contact.dist < self.clearance:
                key = tuple(sorted((g1, g2)))
                item = dict(phase=phase, time_s=float(elapsed), distance_m=float(contact.dist),
                            bodies=[b1, b2], geom_ids=[g1, g2], geoms=[self.model.geom(g1).name or f'geom_{g1}',
                                                  self.model.geom(g2).name or f'geom_{g2}'])
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


def validate_plan(plan, urdf, scene):
    if plan.get('schema_version') != 3 or plan.get('strategy') != 'top_down_direct_ik':
        raise ValueError('Unsupported grasp plan; regenerate an OBB plan')
    import mujoco
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
    if scene.get('left_joint_rad') is not None:
        scene['left_joint_rad'] = _vector(scene['left_joint_rad'], 6, 'left joints').tolist()
    for key in ('lift_joint_m', 'left_finger_opening_m'):
        if scene.get(key) is not None and not np.isfinite(float(scene[key])):
            raise ValueError(f'Invalid {key}')
    model, data = build_scene(urdf, plan, scene)
    screen = CollisionScreen(model, data, scene)
    problems = []
    missing = [key for key in ('table_top_z_m', 'left_joint_rad', 'lift_joint_m', 'left_finger_opening_m')
               if scene.get(key) is None]
    if scene.get('scene_confirmed') is not True:
        missing.append('scene_confirmed')
    samples = 0
    max_fk_error = 0.
    for elapsed, phase, q, grip in trajectory_samples(plan):
        samples += 1
        for i, value in enumerate(q):
            if not joint_lower[i]-1e-6 <= value <= joint_upper[i]+1e-6:
                problems.append(f'{phase}: right joint {i+1} violates hardware limits')
        set_pose(model, data, q, grip)
        error = float(np.max(np.abs(relative_tcp(model, data)-fk(q))))
        max_fk_error = max(max_fk_error, error)
        screen.check(phase, elapsed)
    paths = [('pregrasp',[plan['pregrasp_joint_rad']]),('approach',plan['approach_joint_rad'])]
    for phase, waypoints in paths:
        for q in waypoints:
            for i, value in enumerate(q):
                low, high = joint_lower[i], joint_upper[i]
                if not low+.05-1e-6 <= value <= high-.05+1e-6:
                    problems.append(f'{phase}: planned joint {i+1} violates IK margin')
    if max_fk_error > 1e-5:
        problems.append('MuJoCo TCP differs from calibrated URDF FK')
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
    result = dict(status=status, samples=samples,
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
                               'Carried object, unmodeled obstacles and real tracking/calibration errors are not checked.',
                               'Finger opening uses a linear motor-to-URDF approximation.'])
    return result, model, data

