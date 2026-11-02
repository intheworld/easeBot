"""Robot-owned carried/placed geometry. A model is not proof of physical grasp."""
from copy import deepcopy
import uuid

import numpy as np
from ..skills import digest
from ..grasp.transforms import validate_transforms


def geometry(value):
    value=deepcopy(value)
    size=np.asarray(value['size_m'],dtype=float)
    if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):
        raise ValueError('Payload needs three positive finite dimensions')
    validate_transforms(np.asarray(value['T_tcp_object'],dtype=float)[None],'payload mount')
    return value


def placed_obstacle(payload,footprint_tcp,drop_m=0.):
    if not np.isfinite(drop_m) or drop_m<0:raise ValueError('Invalid settling envelope')
    transform=footprint_tcp@np.asarray(payload['T_tcp_object'])
    drop=np.array([0.,0.,drop_m])
    size=np.asarray(payload['size_m'])+np.abs(transform[:3,:3].T@drop)
    transform[:3,3]-=drop/2
    return dict(id=payload['id'],size_m=size.tolist(),T_footprint_object=transform.tolist(),settling_envelope_m=drop_m)


class CarryState:
    def __init__(self):
        self.status='unknown';self.payload=None;self.obstacles=[];self.reason='Startup gripper state unobserved';self.revision=0
        self.open_rad=None;self.gap_per_rad=None
        self.observation={'jaw_state':'unavailable','gap_m':-1.0,'wrist_state':'unobserved'}

    def configure_gripper(self,open_rad,gap_per_motor_rad_m):
        if not np.isfinite([open_rad,gap_per_motor_rad_m]).all() or open_rad>=0 or gap_per_motor_rad_m>=0:
            raise ValueError('Invalid gripper observation calibration')
        self.open_rad=float(open_rad);self.gap_per_rad=abs(float(gap_per_motor_rad_m))

    def observe(self,state):
        position=state.get('right_gripper')
        if position is None or not np.isfinite(position) or self.gap_per_rad is None:
            self.observation.update(jaw_state='unavailable',gap_m=-1.0)
            return
        fully_open=position<=self.open_rad+.02
        self.observation.update(gap_m=max(0.,-float(position)*self.gap_per_rad),
            jaw_state='open' if fully_open else 'closed_or_partial')
        # Startup is not proof of empty jaws. Only an observed open jaw or a
        # zero-closed encoder with no modeled payload establishes initial empty.
        zero_closed=abs(float(position))<=.01
        if fully_open or (self.payload is None and zero_closed and self.status=='unknown'):
            if self.status!='empty' or self.payload is not None:
                self.payload=None;self.status='empty'
                self.reason='Live gripper feedback: fully open' if fully_open else 'Live gripper feedback: zero closed'
                self._change();self.observation['wrist_state']='unobserved'

    def apply_contact(self,evidence):
        if evidence.get('contact_state')=='closed_without_object':
            if self.status!='empty' or self.payload is not None:
                self.payload=None;self.status='empty';self.reason='Measured closure: no object in jaws'
                self.observation['wrist_state']='unobserved';self._change()
        elif evidence.get('physical_capture_verified') and self.payload is not None and self.status!='attached':
            self.status='attached';self.reason='Measured sustained jaw/load contact'
            self.remove_source_obstacle();self._change()

    def snapshot(self):
        return deepcopy(dict(status=self.status,payload=self.payload,obstacles=self.obstacles,
                             reason=self.reason,revision=self.revision))

    @property
    def token(self):return digest(self.snapshot())

    def _change(self):self.revision+=1

    def require_empty(self):
        if self.status!='empty':raise RuntimeError('Gripper may hold an object; place it or explicitly confirm empty')

    def check(self,state):
        self.observe(state)
        if self.status=='unknown':raise RuntimeError('Carried object state unknown: '+self.reason)
        for obstacle in self.obstacles:
            anchor=obstacle.get('anchor_wheels')
            if anchor is not None and max(abs(a-b) for a,b in zip(state['wheels'],anchor))>.02:
                raise RuntimeError('Base moved since object placement; reconcile the placed-object scene')
        return self.snapshot()

    def uncertain(self,reason,payload=None):
        self.status='unknown';self.reason=reason
        if payload is not None:self.payload=geometry(payload)
        self._change()

    def attach(self,obb,tcp,gripper_rad,*,pending=False):
        object_pose=np.eye(4);object_pose[:3,:3]=obb['rotation'];object_pose[:3,3]=obb['center_m']
        validate_transforms(object_pose[None],'carried OBB')
        self.payload=geometry(dict(id=uuid.uuid4().hex,size_m=obb['size_m'],
            T_tcp_object=(np.linalg.inv(tcp)@object_pose).tolist(),gripper_rad=float(gripper_rad),
            geometry_scope=obb.get('geometry_scope','visible_surface_only'),
            source_obstacle_id=obb.get('known_obstacle_id',''),
            detection_prompt=obb.get('detection_prompt',obb.get('target',''))))
        if not pending:self.remove_source_obstacle()
        self.status='unknown' if pending else 'attached'
        self.reason='Closure in progress; capture unverified' if pending else ''
        self._change()

    def remove_source_obstacle(self):
        source=(self.payload or {}).get('source_obstacle_id','')
        self.obstacles=[o for o in self.obstacles if o['id']!=source]

    def released(self,base_footprint_from_tcp,drop_m=0.,wheels=None):
        if self.payload is None:raise RuntimeError('No carried geometry to release')
        obstacle=placed_obstacle(self.payload,base_footprint_from_tcp,drop_m)
        if wheels is not None:obstacle['anchor_wheels']=list(wheels)
        self.obstacles.append(obstacle)
        self.payload=None;self.status='empty';self.reason='';self._change()
        return obstacle

    def confirm_empty(self):
        self.payload=None;self.status='empty';self.reason='Operator confirmed empty';self._change()

    def clear_obstacles(self):
        self.obstacles=[];self._change()
