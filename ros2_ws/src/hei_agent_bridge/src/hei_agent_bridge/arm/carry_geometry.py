"""Conservative box geometry for carried and known placed objects."""
import numpy as np
from ..grasp.transforms import validate_transforms


def add_geometry(spec,plan,scene):
    import mujoco
    carry=plan.get('carry') or {}
    padding=float(scene.get('payload_padding_m',.003))
    if not np.isfinite(padding) or not 0<=padding<=.03:raise ValueError('Invalid payload padding')
    def add(parent,name,transform,size):
        transform=validate_transforms(np.asarray(transform,dtype=float)[None],name)[0]
        size=np.asarray(size,dtype=float)
        if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):raise ValueError('Invalid object dimensions')
        quat=np.empty(4);mujoco.mju_mat2Quat(quat,transform[:3,:3].ravel())
        body=parent.add_body(name=name,pos=transform[:3,3],quat=quat)
        body.add_geom(name=name+'_box',type=mujoco.mjtGeom.mjGEOM_BOX,size=size/2+padding,
                      rgba=[.8,.3,.1,.5],contype=1,conaffinity=1,group=0)
    payload=carry.get('payload')
    if payload is not None:
        if carry.get('status')!='attached':raise ValueError('Cannot validate unknown payload')
        add(spec.body('a_right_tcp'),'a_right_payload',payload['T_tcp_object'],payload['size_m'])
    obstacles=carry.get('obstacles',[])
    if len(obstacles)>16:raise ValueError('Too many placed objects; reconcile scene')
    for obstacle in obstacles:
        if obstacle.get('kind')=='place_region_coverage':
            boxes=obstacle['boxes']
            if len(boxes)>4096:raise ValueError('Container coverage too fragmented')
            parent=np.asarray(obstacle['T_footprint_object'],float)
            for index,box in enumerate(boxes):
                if box['kind'] not in ('unknown','occupied'):raise ValueError('Invalid coverage exclusion kind')
                add(spec.body('base_footprint'),'placed_'+obstacle['id']+'_'+box['kind']+'_'+str(index),
                    parent@np.asarray(box['T_region_box'],float),box['size_m'])
        else:
            add(spec.body('base_footprint'),'placed_'+obstacle['id'],obstacle['T_footprint_object'],obstacle['size_m'])
