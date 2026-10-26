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
    def __call__(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('Expected six finite calibrated joint angles')
        T, index = np.eye(4), 0
        for j in self.chain:
            A = np.eye(4)
            origin = j.find('origin')
            if origin is not None:
                A[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
                Rx = cv2.Rodrigues(np.array([roll, 0., 0.]))[0]
                Ry = cv2.Rodrigues(np.array([0., pitch, 0.]))[0]
                Rz = cv2.Rodrigues(np.array([0., 0., yaw]))[0]
                A[:3, :3] = Rz @ Ry @ Rx
            T = T @ A
            if j.get('type') == 'revolute':
                B = np.eye(4)
                B[:3, :3] = cv2.Rodrigues(np.fromstring(j.find('axis').get('xyz'), sep=' ') * q[index])[0]
                T = T @ B
                index += 1
            elif j.get('type') != 'fixed':
                raise ValueError('Unexpected movable tool joint')
        return T
