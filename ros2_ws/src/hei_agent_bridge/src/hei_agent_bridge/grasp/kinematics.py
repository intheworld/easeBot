"""URDF forward kinematics for the calibrated HEI right arm, radians/metres."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET
import cv2
import numpy as np

class RightArmFK:
    base_frame = 'a_right_base_link'
    tool_frame = 'a_right_tcp'
    def __init__(self, urdf):
        self.path = Path(urdf).resolve()
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        root = ET.parse(self.path).getroot()
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        link = self.tool_frame
        while link != self.base_frame:
            j = parents[link]
            self.chain.insert(0, j)
            link = j.find('parent').get('link')
        self.names = [j.get('name') for j in self.chain if j.get('type') == 'revolute']
        if self.names != [f'a_right_joint{i}' for i in range(1, 7)]:
            raise ValueError('Unexpected right-arm chain; verify joint-to-motor mapping')
        # Parse fixed geometry once; IK calls FK thousands of times.
        self.transforms = []
        for j in self.chain:
            A = np.eye(4)
            origin = j.find('origin')
            if origin is not None:
                A[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
                A[:3, :3] = (cv2.Rodrigues(np.array([0., 0., yaw]))[0]
                             @ cv2.Rodrigues(np.array([0., pitch, 0.]))[0]
                             @ cv2.Rodrigues(np.array([roll, 0., 0.]))[0])
            axis = None
            if j.get('type') == 'revolute':
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
            elif j.get('type') != 'fixed':
                raise ValueError('Unexpected movable tool joint')
            self.transforms.append((A, axis))

    def __call__(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Expected six finite calibrated joint angles')
        T, index = np.eye(4), 0
        for A, axis in self.transforms:
            T = T @ A
            if axis is not None:
                B = np.eye(4)
                B[:3, :3] = cv2.Rodrigues(axis * q[index])[0]
                T = T @ B
                index += 1
        return T

    def with_jacobian(self,q):
        """TCP pose and spatial geometric Jacobian in the calibrated base."""
        q=np.asarray(q,float)
        if q.shape!=(6,) or not np.isfinite(q).all():
            raise ValueError('Expected six finite calibrated joint angles')
        t=np.eye(4);axes=[];origins=[];index=0
        for local,axis in self.transforms:
            t=t@local
            if axis is not None:
                origins.append(t[:3,3].copy());axes.append(t[:3,:3]@axis)
                rotation=np.eye(4);rotation[:3,:3]=cv2.Rodrigues(axis*q[index])[0]
                t=t@rotation;index+=1
        axes=np.asarray(axes);origins=np.asarray(origins)
        jac=np.vstack([np.cross(axes,t[:3,3]-origins).T,axes.T])
        return t,jac

    def batch(self,qs):
        """Exact URDF FK for independent joint configurations in one NumPy batch."""
        qs=np.asarray(qs,float)
        if qs.ndim!=2 or qs.shape[1]!=6 or not np.isfinite(qs).all():
            raise ValueError('Expected finite batched calibrated joint angles')
        t=np.repeat(np.eye(4)[None],len(qs),axis=0);index=0
        for local,axis in self.transforms:
            t=t@local
            if axis is not None:
                length=float(np.linalg.norm(axis))
                unit=axis/length if length else axis
                x,y,z=unit;skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
                angle=qs[:,index]*length
                rotation=np.eye(3)+np.sin(angle)[:,None,None]*skew+(1-np.cos(angle))[:,None,None]*(skew@skew)
                t[:,:3,:3]=t[:,:3,:3]@rotation;index+=1
        return t
