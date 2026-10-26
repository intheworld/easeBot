"""Gravity-aligned OBB and HEI top-down candidates, in meters and base frame.

Inspired by GaP's perceive -> target_obb -> align -> descend decomposition.
A single RGB-D view bounds VISIBLE surfaces only; it does not recover hidden
object geometry or prove support height, object identity, or successful grasp.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

from ..arm.geometry import base_from_frame, origin_matrix
from .transforms import validate_transforms


def visible_obb(points, up):
    points = np.asarray(points, dtype=float)
    up = np.asarray(up, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 30 or not np.isfinite(points).all():
        raise ValueError('OBB needs at least 30 finite visible surface points')
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < .99:
        raise ValueError('Invalid gravity direction')
    up = up / np.linalg.norm(up)
    x = np.array([1., 0., 0.]); x -= np.dot(x, up)*up
    if np.linalg.norm(x) < .1:
        x = np.array([0., 1., 0.]); x -= np.dot(x, up)*up
    x /= np.linalg.norm(x)
    basis = np.column_stack((x, np.cross(up,x), up))
    origin = np.median(points, axis=0)
    local = (points-origin) @ basis
    # Reject isolated RGB-D speckles, then use the full remaining bounds.
    low, high = np.quantile(local,[.01,.99],axis=0)
    retained = np.all((local >= low-1e-6) & (local <= high+1e-6),axis=1)
    local = local[retained]
    if len(local) < 30:
        raise ValueError('Too few valid points after depth outlier filtering')
    rect = cv2.minAreaRect(np.asarray(local[:,:2],dtype=np.float32))
    corners = cv2.boxPoints(rect).astype(float)
    edges = [corners[1]-corners[0], corners[2]-corners[1]]
    long = max(edges,key=np.linalg.norm)
    if np.linalg.norm(long) < 1e-6:
        raise ValueError('Degenerate OBB footprint')
    long /= np.linalg.norm(long)
    if long[0] < 0 or (abs(long[0]) < 1e-8 and long[1] < 0):long = -long
    axis = basis[:,:2] @ long
    rotation = np.column_stack((axis,np.cross(up,axis),up))
    cloud = points[retained]
    projected = (cloud-origin) @ rotation
    lower, upper = projected.min(axis=0), projected.max(axis=0)
    size = upper-lower
    if np.any(size[:2] < 1e-5):
        raise ValueError('Degenerate OBB footprint')
    center = origin+rotation @ ((lower+upper)/2)
    return {'schema':'hei.obb/v1','frame_id':'a_right_base_link','center_m':center.tolist(),
            'size_m':size.tolist(),'rotation':rotation.tolist(),'point_count':len(cloud),
            'geometry_scope':'visible_surface_only','up_axis':up.tolist()}


def gripper_geometry(urdf, profile):
    """Derive closing axis and forward finger envelope from this URDF's CAD.

    TCP +X is HEI's declared approach axis. Prismatic finger axis and CAD
    envelope are transformed into TCP coordinates, instead of copying Panda
    offsets or treating the hand frame as a fingertip frame.
    """
    urdf = Path(urdf)
    root = ET.parse(urdf).getroot()
    joints = {j.find('child').get('link'):j for j in root.findall('joint')}
    def relative(link):
        if link == 'a_right_link6':return np.eye(4)
        joint = joints[link]
        if joint.get('type') not in ('fixed','prismatic'):
            raise ValueError('Unexpected moving gripper joint')
        return relative(joint.find('parent').get('link')) @ origin_matrix(joint)
    tcp_inverse = np.linalg.inv(relative('a_right_tcp'))
    axes, reach = [], 0.
    for name in ('a_right_end_link_L_finger','a_right_end_link_R_finger'):
        transform = tcp_inverse @ relative(name)
        axis = np.fromstring(joints[name].find('axis').get('xyz'),sep=' ')
        axes.append(transform[:3,:3] @ axis)
        link = root.find(f"link[@name='{name}']")
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:raise ValueError('Finger envelope needs collision mesh')
            path = urdf.parent.parent/'meshes'/Path(mesh.get('filename')).name
            raw = path.read_bytes()
            count = int.from_bytes(raw[80:84],'little')
            if len(raw) != 84+50*count:raise ValueError('Expected binary STL finger geometry')
            dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
            vertices = np.frombuffer(raw,dtype=dtype,count=count,offset=84)['vertices'].reshape(-1,3).astype(float)
            vertices *= np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            t = transform @ origin_matrix(collision)
            forward = (vertices @ t[:3,:3].T+t[:3,3])[:,0]
            reach = max(reach,float(np.max(forward)))
    close_axis = axes[0]/np.linalg.norm(axes[0])
    if not np.allclose(abs(np.dot(close_axis,[0,1,0])),1,atol=1e-4):
        raise ValueError('Unexpected HEI finger axis; recheck TCP convention')
    gap = -profile['gripper_open_rad']*abs(profile['gap_per_motor_rad_m'])
    return {'tool_frame':'a_right_tcp','approach_axis':[1.,0.,0.],
            'closing_axis':close_axis.tolist(),'finger_forward_m':reach,'open_gap_m':gap}


def top_down_candidates(obb, geometry, profile, support_z):
    if obb.get('schema') != 'hei.obb/v1' or obb.get('frame_id') != 'a_right_base_link':
        raise ValueError('OBB schema/frame mismatch')
    center, size, rotation = (np.asarray(obb[k],dtype=float) for k in ('center_m','size_m','rotation'))
    transform = np.eye(4);transform[:3,:3] = rotation
    validate_transforms(transform[None],'OBB')
    if center.shape != (3,) or size.shape != (3,) or not np.isfinite([center,size]).all() or np.any(size<0):
        raise ValueError('Invalid OBB dimensions')
    up = rotation[:,2]
    top = float(center @ up+size[2]/2)
    # A shallow configurable pinch below the observed top; never place the CAD
    # fingertip envelope below the already-configured physical support plane.
    depth = min(profile['topdown_grip_depth_m'],size[2]/2)
    height = max(top-depth,support_z+geometry['finger_forward_m']+profile['finger_clearance_m'])
    if height > top+.003:
        raise ValueError('Observed target is too low for the finger/support clearance')
    point = center+up*(height-float(center @ up))
    if profile.get('min_base_z_m') is not None and point[2] < profile['min_base_z_m']:
        raise ValueError('Grasp below explicitly configured base-frame height')
    candidates = []
    # Prefer closing over the short horizontal axis. Also try its symmetric
    # wrist and the long-axis pinch when the measured width fits the opening.
    closing_axes = [rotation[:,1],-rotation[:,1],rotation[:,0],-rotation[:,0]]
    for angle in (-np.pi/12,np.pi/12):
        axis = np.cos(angle)*rotation[:,1]+np.sin(angle)*rotation[:,0]
        closing_axes.extend((axis,-axis))
    for close in closing_axes:
        width = float(np.abs(rotation.T @ close) @ size)
        if width+profile['jaw_clearance_m'] > geometry['open_gap_m']:continue
        goal = np.eye(4)
        goal[:3,:3] = np.column_stack((-up,close,np.cross(-up,close)))
        goal[:3,3] = point
        pre = goal.copy();pre[:3,3] += up*profile['pregrasp_offset_m']
        candidates.append({'T_grasp':goal.tolist(),'T_pregrasp':pre.tolist(),'jaw_width_m':width})
    if not candidates:raise ValueError('No OBB candidate fits the configured gripper opening')
    return candidates


def support_geometry(urdf, state, scene):
    if scene.get('table_top_z_m') is None:
        raise ValueError('Top-down grasp needs the configured physical support plane')
    frame = base_from_frame(urdf,state,scene.get('table_frame','a_right_base_link'))
    up = frame[:3,2]
    point = frame @ np.array([0.,0.,float(scene['table_top_z_m']),1.])
    return up,float(point[:3] @ up)


def segmented_cloud(results, depth, intrinsic, target, camera_to_base, diagnostics=None):
    from .common_utils import tensor_to_numpy
    depth = np.asarray(depth,dtype=float)
    k = np.asarray(intrinsic,dtype=float).reshape(3,3)
    if depth.ndim != 2 or not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0:
        raise ValueError('Invalid aligned depth/intrinsics')
    candidates = []
    report = diagnostics if diagnostics is not None else {}
    report.update(target=target, detections=0, matching_detections=0, segmented_detections=0,
                  objects=[])
    for result in results:
        boxes,masks = getattr(result,'boxes',None),getattr(result,'masks',None)
        if boxes is None:continue
        classes, confidence = (tensor_to_numpy(v) for v in (boxes.cls,boxes.conf))
        segments = None if masks is None else tensor_to_numpy(masks.data)
        if classes is None or confidence is None:continue
        for index,(category,score) in enumerate(zip(classes.ravel(),confidence.ravel())):
            name = result.names[int(category)]
            report['detections'] += 1
            if name != target or not np.isfinite(score) or not 0 <= score <= 1:continue
            report['matching_detections'] += 1
            if segments is None or index >= len(segments):continue
            report['segmented_detections'] += 1
            mask = cv2.resize(np.asarray(segments[index],dtype=np.float32),(depth.shape[1],depth.shape[0]),
                              interpolation=cv2.INTER_NEAREST) > .5
            v,u = np.nonzero(mask & np.isfinite(depth) & (depth>0))
            report['objects'].append(dict(confidence=float(score), mask_pixels=int(mask.sum()),
                                          valid_depth_pixels=int(len(u))))
            if len(u) < 30:continue
            selected = np.linspace(0,len(u)-1,min(20000,len(u)),dtype=int)
            u,v = u[selected],v[selected]
            z = depth[v,u]
            camera = np.column_stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z))
            world = camera @ camera_to_base[:3,:3].T+camera_to_base[:3,3]
            candidates.append((float(score),world,mask))
    if not candidates:
        if not report['matching_detections']:
            reason = 'detector returned no matching detections'
        elif not report['segmented_detections']:
            reason = 'matching detections have no segmentation masks'
        else:
            reason = 'segmentation masks contain fewer than 30 valid depth pixels'
        raise ValueError(f'No segmented RGB-D object for target {target!r}: {reason}; inspect perception.json')
    return max(candidates,key=lambda item:item[0])
