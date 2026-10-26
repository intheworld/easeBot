"""Strict homogeneous-transform validation shared by IK and OBB geometry."""
import numpy as np


def validate_transforms(values, name):
    values = np.asarray(values, dtype=np.float64)
    if values.ndim != 3 or values.shape[1:] != (4, 4) or not np.isfinite(values).all():
        raise ValueError(f'{name} must be finite N x 4 x 4 transforms')
    for T in values:
        if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
            raise ValueError(f'{name}: invalid homogeneous last row')
        if not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-5):
            raise ValueError(f'{name}: invalid rotation')
    return values
