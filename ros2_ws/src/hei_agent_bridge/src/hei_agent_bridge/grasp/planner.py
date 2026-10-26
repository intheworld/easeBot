"""IK and Cartesian samples migrated from rebot_grasp; no hardware imports."""
import cv2
import numpy as np
from scipy.optimize import least_squares
from .kinematics import RightArmFK
from .transforms import validate_transforms
from .validation import right_arm_joint_limits

class RightArmPlanner:
    def __init__(self, urdf, margin_rad=0.05):
        self.fk = RightArmFK(urdf)
        self.hardware_lower, self.hardware_upper = right_arm_joint_limits(self.fk)
        self.lower = self.hardware_lower + margin_rad
        self.upper = self.hardware_upper - margin_rad

    def solve(self, target, seed):
        target = validate_transforms(np.asarray(target)[None], 'IK target')[0]
        seed = np.asarray(seed, dtype=float)
        if seed.shape != (6,) or not np.isfinite(seed).all():
            raise ValueError('IK needs six finite seed angles')

        def residual(q):
            current = self.fk(q)
            return np.r_[current[:3, 3] - target[:3, 3],
                         .15 * cv2.Rodrigues(target[:3, :3].T @ current[:3, :3])[0].ravel()]

        result = least_squares(residual, np.clip(seed, self.lower, self.upper),
                               bounds=(self.lower, self.upper), max_nfev=250,
                               ftol=1e-10, xtol=1e-10, gtol=1e-10)
        actual = self.fk(result.x)
        position_error = np.linalg.norm(actual[:3, 3] - target[:3, 3])
        angle_error = np.linalg.norm(cv2.Rodrigues(target[:3, :3].T @ actual[:3, :3])[0])
        if position_error > .005 or angle_error > np.deg2rad(3):
            raise ValueError(f'Right-arm IK failed: {position_error*1000:.1f} mm, '
                             f'{np.degrees(angle_error):.1f} deg')
        return result.x

    def cartesian_path(self, start_q, goal, step_m=.005):
        """Preflight a straight TCP approach/retreat, including every IK sample."""
        goal = validate_transforms(np.asarray(goal)[None], 'Cartesian target')[0]
        if not np.isfinite(step_m) or step_m <= 0:
            raise ValueError('Cartesian step must be positive')
        start = self.fk(start_q)
        delta_rotation = cv2.Rodrigues(goal[:3, :3] @ start[:3, :3].T)[0].ravel()
        distance = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
        if not np.isfinite(distance) or distance/step_m > 1024:
            raise ValueError('Cartesian path exceeds 1024 sample budget')
        steps = max(1, int(np.ceil(distance / step_m)),
                    int(np.ceil(np.linalg.norm(delta_rotation) / .05)))
        path = []
        q = np.asarray(start_q).copy()
        for alpha in np.linspace(0, 1, steps + 1)[1:]:
            target = np.eye(4)
            target[:3, 3] = (1-alpha)*start[:3, 3] + alpha*goal[:3, 3]
            target[:3, :3] = cv2.Rodrigues(alpha*delta_rotation)[0] @ start[:3, :3]
            next_q = self.solve(target, q)
            if np.max(np.abs(next_q-q)) > .2:
                raise ValueError('IK branch jump in Cartesian path')
            path.append(next_q)
            q = next_q
        return path
