"""Small shared helpers used by perception utilities."""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

def tensor_to_numpy(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def grasp_target(value: str) -> str:
    """One bounded open-vocabulary object description, never a class whitelist."""
    if type(value) is not str or not value.strip() or len(value) > 160 or any(ord(c) < 32 or ord(c) == 127 for c in value):
        raise ValueError('Grasp target must be a nonempty object description of at most 160 characters without control characters')
    return value.strip()
