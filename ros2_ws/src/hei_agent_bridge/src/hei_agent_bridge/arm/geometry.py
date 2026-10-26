"""Explicit URDF frame transforms and scalar TCP poses; meters, radians, xyzw."""
import math
import xml.etree.ElementTree as ET

import cv2
import numpy as np

FRAMES = ('a_right_base_link', 'base_link', 'base_footprint')


def quaternion_matrix(values):
    q = np.asarray(values, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .001:
        raise ValueError('Quaternion must be finite unit xyzw (norm tolerance 0.001)')
    q = q/np.linalg.norm(q)
    length = float(np.linalg.norm(q[:3]))
    if length < 1e-12:
        return np.eye(3)
    return cv2.Rodrigues(q[:3]/length * (2*math.atan2(length, q[3])))[0]


def matrix_quaternion(rotation):
    vector = cv2.Rodrigues(np.asarray(rotation, dtype=float))[0].ravel()
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return [0., 0., 0., 1.]
    q = np.r_[vector/angle*math.sin(angle/2), math.cos(angle/2)]
    if q[3] < 0:
        q = -q
    return [float(v) for v in q/np.linalg.norm(q)]


def origin_matrix(joint):
    result = np.eye(4)
    origin = joint.find('origin')
    if origin is not None:
        result[:3,3] = np.fromstring(origin.get('xyz','0 0 0'), sep=' ')
        r,p,y = np.fromstring(origin.get('rpy','0 0 0'), sep=' ')
        result[:3,:3] = (cv2.Rodrigues(np.array([0.,0.,y]))[0] @
                         cv2.Rodrigues(np.array([0.,p,0.]))[0] @
                         cv2.Rodrigues(np.array([r,0.,0.]))[0])
    return result


def base_from_frame(urdf, state, frame):
    if frame not in FRAMES:
        raise ValueError('Unsupported target frame')
    if frame == 'a_right_base_link':
        return np.eye(4)
    parents = {j.find('child').get('link'):j for j in ET.parse(urdf).getroot().findall('joint')}
    def root_from(link):
        chain, seen = [], set()
        while link in parents:
            if link in seen:
                raise ValueError('URDF cycle')
            seen.add(link)
            joint = parents[link]
            chain.insert(0, joint)
            link = joint.find('parent').get('link')
        result = np.eye(4)
        for j in chain:
            result = result @ origin_matrix(j)
            if j.get('type') == 'fixed':
                continue
            if j.get('name') != 'lift_joint' or j.get('type') != 'prismatic':
                raise ValueError('Frame conversion crosses an unsupported movable joint')
            lift = float(state['lift'])
            if not math.isfinite(lift):
                raise ValueError('Invalid lift feedback')
            axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
            shift = np.eye(4); shift[:3,3] = axis*lift
            result = result @ shift
        return link, result
    root, base = root_from('a_right_base_link')
    source_root, source = root_from(frame)
    if root != source_root:
        raise ValueError('Disconnected URDF frames')
    return np.linalg.inv(base) @ source


def pose_fields(transform, frame):
    q = matrix_quaternion(transform[:3,:3])
    return {'side':'right', 'frame_id':frame, 'tool_frame':'a_right_tcp',
            **dict(zip(('x_m','y_m','z_m'), [float(v) for v in transform[:3,3]])),
            **dict(zip(('qx','qy','qz','qw'), q))}


def target_transform(arguments, current_in_frame):
    result = current_in_frame.copy()
    xyz = [arguments[k] for k in ('x_m','y_m','z_m')]
    if not np.isfinite(xyz).all():
        raise ValueError('Target position must be finite meters')
    result[:3,3] = xyz
    if all(k in arguments for k in ('qx','qy','qz','qw')):
        result[:3,:3] = quaternion_matrix([arguments[k] for k in ('qx','qy','qz','qw')])
    return result
