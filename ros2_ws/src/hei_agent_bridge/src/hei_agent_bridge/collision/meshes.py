"""Version-keyed CAD mesh decomposition shared by online and offline colliders."""
from functools import lru_cache
from pathlib import Path
import numpy as np


def stl_components(path):
    """Partition disconnected binary STL solids without excluding any geometry."""
    source = Path(path).resolve()
    stat = source.stat()
    return _stl_components(str(source), stat.st_mtime_ns, stat.st_size)

@lru_cache(maxsize=32)
def _stl_components(path, mtime_ns, size):
    # File-version-keyed geometry only; collision results are never cached.
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    raw = Path(path).read_bytes()
    if len(raw) < 84:
        raise ValueError('Unsupported STL')
    count = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84+50*count:
        raise ValueError('Expected binary STL')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    points = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    vertices, inverse = np.unique(points, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    a, b = faces.ravel(), faces[:, [1, 2, 0]].ravel()
    graph = coo_matrix((np.ones(len(a)), (a, b)), shape=(len(vertices), len(vertices)))
    number, labels = connected_components(graph, directed=False)
    pieces = [vertices[labels == i].astype(float) for i in range(number)]
    if any(len(v) < 4 or np.linalg.matrix_rank(v-v.mean(0)) < 3 for v in pieces):
        # Never discard degenerate fragments; keep the original collider instead.
        pieces = [vertices.astype(float)]
    for piece in pieces:
        piece.setflags(write=False)
    return tuple(pieces)
