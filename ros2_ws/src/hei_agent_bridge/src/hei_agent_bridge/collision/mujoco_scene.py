"""Explicit offline MuJoCo scene construction and pose evaluation.

Importing this module does not load MuJoCo or initialize hardware.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .meshes import stl_components
from ..grasp.transforms import validate_transforms


def _vector(value, count, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (count,) or not np.isfinite(value).all():
        raise ValueError(f'{name}: expected {count} finite numbers')
    return value

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
        base.add_site(name=f'validation_{name}', pos=target[:3, 3], size=[.008,0.,0.], rgba=color)
        for axis in range(3):
            endpoint = target[:3, 3]+target[:3, axis]*.06
            rgba = [0., 0., 0., 1.]; rgba[axis] = 1.
            base.add_site(name=f'validation_{name}_axis{axis}', type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                          fromto=np.r_[target[:3, 3], endpoint], size=[.002,0.,0.], rgba=rgba)
    from ..arm.carry_geometry import add_geometry
    add_geometry(spec,plan,scene)
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
