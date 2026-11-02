"""Warm process-local planning caches using synthetic poses, never robot I/O.

The result is diagnostics only. It is not a plan, scene observation, validation,
or permission to execute anything. Synthetic transforms are discarded.
"""
import time
import numpy as np


def warmup(request):
    from .grasp.planner_backend import get_planner
    from .collision.fcl_scene import robot_model
    timings={};started=time.monotonic();last=started
    def phase(name):
        nonlocal last
        now=time.monotonic();timings[name]=now-last;last=now
    urdf=request['urdf'];profile=request['profile']
    planner=get_planner(urdf,profile.get('ik_backend','auto'))
    phase('planner_init_sec')
    q=np.clip(np.array([.3,-1.,-1.,.5,.1,.2]),planner.lower,planner.upper)
    if getattr(planner,'backend','scipy')=='curobo':
        planner.proposals(np.array([planner.fk(q)]),q)
    else:
        planner.solve(planner.fk(q),q)
    phase('first_ik_sec')
    robot_model(urdf)
    phase('collision_meshes_sec')
    accelerator='cpu_fcl'
    if getattr(planner,'backend','scipy')=='curobo':
        import os
        if os.environ.get('HEI_COLLISION_ACCELERATOR','auto')!='cpu':
            from .collision.fcl_scene import FCLScene
            from .collision.cuda_batch import CudaBatch
            # Empty synthetic scene: no feedback, payload, or collision result
            # survives this function. Only immutable geometry/kernel caches warm.
            scene=FCLScene(urdf,{},profile['scene'])
            batch=CudaBatch(scene)
            batch.compute(np.repeat(q[None],32,axis=0),np.full(32,-2.25))
            accelerator='cuda_fk_aabb_sat+fcl'
    phase('collision_cuda_sec')
    return dict(operation='warmup',status='ready',ik_backend=getattr(planner,'backend','scipy'),
                collision_accelerator=accelerator,timings=timings,
                elapsed_sec=time.monotonic()-started,synthetic=True,motion_sent=False)
