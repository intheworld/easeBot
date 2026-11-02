"""Process-local planner reuse; never shares mutable state between processes."""
from functools import lru_cache
from pathlib import Path
import importlib.util


def get_planner(urdf, backend='auto'):
    if backend not in ('auto','scipy','curobo'):
        raise ValueError('ik_backend must be auto, scipy or curobo')
    path=Path(urdf).resolve()
    return _planner(str(path),path.stat().st_mtime_ns,backend)


@lru_cache(maxsize=4)
def _planner(path,mtime,backend):
    use_cuda=False
    if backend!='scipy' and importlib.util.find_spec('curobo') is not None:
        import torch
        use_cuda=torch.cuda.is_available()
    if use_cuda:
        from .gpu_planner import CudaRightArmPlanner
        return CudaRightArmPlanner(path)
    if backend=='curobo':raise RuntimeError('cuRobo backend requested but CUDA/cuRobo unavailable')
    from .planner import RightArmPlanner
    return RightArmPlanner(path)


def cached_cartesian_path(planner,start,goal,*,strict_intermediate=True,strict_endpoint=True):
    import numpy as np
    start=np.asarray(start,dtype=float);goal=np.asarray(goal,dtype=float)
    key=(start.tobytes(),goal.tobytes(),bool(strict_intermediate),bool(strict_endpoint))
    cache=getattr(planner,'_path_cache',None)
    if cache is None:
        from collections import OrderedDict
        planner._path_cache=cache=OrderedDict()
    if key not in cache:
        try:
            cache[key]=(planner.cartesian_path(start,goal,strict_intermediate=strict_intermediate,strict_endpoint=strict_endpoint),None)
        except ValueError as error:
            cache[key]=(None,str(error))
        if len(cache)>128:cache.popitem(last=False)
    path,error=cache[key]
    cache.move_to_end(key)
    if error is not None:raise ValueError(error)
    return [q.copy() for q in path]
