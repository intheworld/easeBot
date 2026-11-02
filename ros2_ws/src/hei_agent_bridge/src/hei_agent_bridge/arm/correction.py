"""Bounded endpoint recovery using freshly validated trajectories."""
import asyncio
import json
import math
import time
from copy import deepcopy

import cv2
import numpy as np

from .geometry import pose_fields


async def correct_endpoint(service, plan, invocation, directory):
    target=np.asarray(plan['T_goal'],float)
    anchor=service.state(at_rest=False)
    # Reconcile fresh jaw feedback before freezing the scene revision.
    service.carry.check(anchor)
    carry_token=service.carry.token
    records=[]
    directory.mkdir(parents=True,exist_ok=True)

    def check_scene(state):
        if (not service.scene_matches(anchor,state) or
                abs(anchor['right_gripper']-state['right_gripper'])>.02 or service.carry.token!=carry_token):
            raise RuntimeError('Scene/gripper changed during endpoint correction')

    async def observe():
        deadline=time.monotonic()+1.
        while True:
            check_scene(service.state(at_rest=False))
            try:
                state=service.state()
                break
            except RuntimeError as error:
                # Only transient right-arm velocity is retryable. Stale data,
                # faults and changes to other scene elements still propagate.
                if str(error)!='Grasp scene is moving: right':raise
                (directory/'settling.json').write_text(json.dumps(dict(status='waiting',reason=str(error))))
                if time.monotonic()>=deadline:
                    raise RuntimeError('Right arm did not settle within 1s before endpoint correction') from error
                await asyncio.sleep(.05)
        check_scene(state)
        actual=service.fk(state['right'])
        delta=actual[:3,3]-target[:3,3]
        angle=float(np.linalg.norm(cv2.Rodrigues(target[:3,:3].T@actual[:3,:3])[0]))
        error=float(np.linalg.norm(delta))
        joint_error=float(np.max(np.abs(np.asarray(state['right'])-plan['waypoints_rad'][-1])))
        if not np.isfinite([error,angle,joint_error]).all():raise RuntimeError('Non-finite endpoint feedback')
        records.append(dict(actual_xyz_m=actual[:3,3].tolist(),target_xyz_m=target[:3,3].tolist(),
                            actual_minus_target_xyz_m=delta.tolist(),frame_id='a_right_base_link',
                            joint_rad=state['right'],position_error_m=error,orientation_error_rad=angle))
        (directory/'feedback.json').write_text(json.dumps(records,allow_nan=False))
        return state,max(error/.005,angle/math.radians(3),joint_error/.03)

    # A controller may report success under joint tolerances before the stricter
    # TCP endpoint has settled. Wait only after an actual endpoint mismatch.
    async def settled():
        state,score=await observe()
        if score<=1:return state,score
        await asyncio.sleep(.5)
        return await observe()

    state,score=await settled()
    for attempt in range(2):
        if score<=1:return state,plan
        path=directory/str(attempt+1);path.mkdir()
        request=dict(directory=str(path),urdf=service.urdf,state=deepcopy(state),profile=service.profile,
                     carry=service.carry.check(state),arguments={**pose_fields(target,'a_right_base_link'),
                     'path_mode':'linear'})
        (path/'request.json').write_text(json.dumps(request,allow_nan=False))
        data=await service._worker(path)
        if data['validation']['status']!='passed':
            raise RuntimeError('Endpoint correction path failed validation; no correction dispatched')
        current,_=await observe()
        if not service.same_start(state,current):raise RuntimeError('Arm moved while planning endpoint correction')
        segment=data['plan']
        async def move():
            await service.backend.arm_trajectory(invocation,current['right'],segment['waypoints_rad'],segment['joint_speed_rad_s'])
        await service.guarded(move,lambda after:service.scene_matches(anchor,after) and
                              abs(anchor['right_gripper']-after['right_gripper'])<=.02,
                              message='Scene/gripper changed during endpoint correction')
        # Use the correction's final joint solution but retain the original TCP
        # target. Equivalent IK branches must not be compared to old joint goals.
        plan={**plan,'waypoints_rad':segment['waypoints_rad']}
        previous=score
        state,score=await settled()
        if score>1 and score>=previous:
            raise RuntimeError('Endpoint correction did not reduce error; see endpoint feedback.json')
    if score>1:raise RuntimeError('Endpoint still outside tolerance after two corrections; see endpoint feedback.json')
    return state,plan
