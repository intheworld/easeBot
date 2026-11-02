"""Placement opening hold checks and bounded feedback settling; no motion planning."""
import asyncio
import json
import time
import numpy as np


class ReleaseMonitor:
    def __init__(self,service,start,endpoint,directory):
        self.service,self.start,self.endpoint=service,start,endpoint
        self.path=directory/'release_feedback.json'
        self.started=time.monotonic();self.rows=[]

    def check(self,state,phase='opening'):
        service=self.service
        row=dict(elapsed_sec=time.monotonic()-self.started,phase=phase,
                 right_joint_rad=list(state['right']),right_gripper_rad=state['right_gripper'],
                 right_joint_delta_rad=(np.asarray(state['right'])-self.start['right']).tolist(),
                 wheel_delta_rad=(np.asarray(state['wheels'])-self.start['wheels']).tolist(),
                 lift_delta_m=state['lift']-self.start['lift'])
        try:
            if not service.scene_matches(self.start,state):
                raise RuntimeError('Scene changed: '+json.dumps({k:row[k] for k in ('wheel_delta_rad','lift_delta_m')}))
            # Reuse the endpoint criteria that already authorized opening:
            # 5 mm TCP / 3 degrees / 0.03 rad final joint error. A small encoder
            # change alone does not imply the release pose is unsafe.
            actual=service.fk(state['right']);target=np.asarray(self.endpoint['T_goal'])
            row['actual_xyz_m']=actual[:3,3].tolist()
            row['actual_minus_target_xyz_m']=(actual[:3,3]-target[:3,3]).tolist()
            position,angle,joint=service.verify_endpoint(self.endpoint,state)
            row.update(position_error_m=position,orientation_error_rad=angle,joint_error_rad=joint,status='passed')
        except Exception as exc:
            row.update(status='failed',reason=str(exc))
            raise RuntimeError(f'Release hold check failed: {exc}; details: {self.path}') from exc
        finally:
            self.rows.append(row);self.path.write_text(json.dumps(self.rows,allow_nan=False))
        return True


async def confirm_opening(service,invocation,gap,monitor,confirmed):
    await service.backend.gripper(invocation,'right',gap)
    state=service.state(at_rest=False)
    expected=-gap/abs(service.profile['gap_per_motor_rad_m'])
    def check_gap(state):
        if abs(state['right_gripper']-expected)>.02:
            raise RuntimeError(f'Release feedback differs from planned opening: actual_rad={state["right_gripper"]}, expected_rad={expected}')
    check_gap(state)
    # Opening confirmation is distinct from the subsequent pose/retreat checks.
    confirmed()
    deadline=time.monotonic()+1.
    while True:
        monitor.check(state,'settling')
        try:
            state=service.state()
            monitor.check(state,'confirmed')
            check_gap(state)
            return state
        except RuntimeError as exc:
            if str(exc)!='Grasp scene is moving: right':raise
            if time.monotonic()>=deadline:
                raise RuntimeError('Opening confirmed, but right arm did not settle within 1s; retreat not started') from exc
            await asyncio.sleep(.05)
            state=service.state(at_rest=False)
