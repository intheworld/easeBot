"""Polytope data and independent separating-axis audit (no collision library).

SAT uses face normals and edge cross products. Its positive separation is a
lower bound on Euclidean distance; it must not replace positive-clearance distance
queries. Negative values give the minimum translational overlap for convex solids.
"""
from itertools import product
import numpy as np
from scipy.spatial import ConvexHull


def directions(rows):
    rows=np.asarray(rows,float).reshape(-1,3)
    length=np.linalg.norm(rows,axis=1)
    rows=rows[length>1e-12]/length[length>1e-12,None]
    # Parallel and antiparallel axes are equivalent. Rounding only merges axes;
    # omitted near-parallel axes make a separation screen more conservative.
    pivot=np.argmax(np.abs(rows),axis=1)
    rows*=np.where(rows[np.arange(len(rows)),pivot]<0,-1.,1.)[:,None]
    _,indices=np.unique(np.round(rows,10),axis=0,return_index=True)
    return rows[np.sort(indices)]


def polytope(points, hull=None):
    points=np.asarray(points,float)
    hull=ConvexHull(points) if hull is None else hull
    # Retaining triangulation diagonals is valid (extra projection axes cannot
    # reject intersecting convex sets); deduplication bounds GPU work.
    edges=np.concatenate([hull.simplices[:,[0,1]],hull.simplices[:,[1,2]],hull.simplices[:,[2,0]]])
    result=dict(kind='convex',vertices=points[hull.vertices].copy(),
                normals=directions(hull.equations[:,:3]),
                edges=directions(points[edges[:,1]]-points[edges[:,0]]))
    for value in result.values():
        if isinstance(value,np.ndarray):value.setflags(write=False)
    return result


class LazyPolytope(dict):
    """Prepare SAT axes only for hulls actually used by the GPU or an audit."""
    def __init__(self,points,hull):
        super().__init__(kind='convex')
        self._points=points;self._hull=hull

    def __getitem__(self,key):
        if key not in self and key in ('vertices','normals','edges'):
            self.update(polytope(self._points,self._hull))
            del self._points,self._hull
        return super().__getitem__(key)


def box_polytope(size):
    return dict(kind='box',vertices=np.array(list(product([-1.,1.],repeat=3)))*np.asarray(size)/2,
                normals=np.eye(3),edges=np.eye(3))


def separating_gap(a,ta,b,tb):
    """Independent CPU audit of two world-transformed convex polytopes."""
    va=a['vertices']@ta[:3,:3].T+ta[:3,3]
    vb=b['vertices']@tb[:3,:3].T+tb[:3,3]
    ea=a['edges']@ta[:3,:3].T;eb=b['edges']@tb[:3,:3].T
    axes=directions(np.vstack([a['normals']@ta[:3,:3].T,b['normals']@tb[:3,:3].T,
                                np.cross(ea[:,None],eb[None,:]).reshape(-1,3)]))
    pa=va@axes.T;pb=vb@axes.T
    gaps=np.maximum(pa.min(0)-pb.max(0),pb.min(0)-pa.max(0))
    i=int(np.argmax(gaps))
    return dict(gap_m=float(gaps[i]),axis=axes[i].tolist(),axes=len(axes))
