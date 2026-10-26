"""Robot-owned TCP pose skills. Planning never sends a ROS goal."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import uuid

import cv2
import numpy as np

from ..grasp.kinematics import RightArmFK
from ..motion import MotionRuntime, data_directory
from .geometry import FRAMES, base_from_frame, pose_fields


def prepare_config(overrides):
    if type(overrides) is not dict:
        raise ValueError('arm_motion must be an object of profile overrides')
    profile = json.loads((data_directory()/'arm_motion.json').read_text())
    if set(overrides)-set(profile):
        raise ValueError('Unknown arm_motion settings')
    profile.update(deepcopy(overrides))
    profile['worker_python'] = str(Path(profile['worker_python']).expanduser().resolve(strict=True))
    if not os.access(profile['worker_python'],os.X_OK):
        raise ValueError('Motion worker Python is not executable')
    speed = profile['joint_speed_rad_s']
    if type(speed) not in (int,float) or not math.isfinite(speed) or not 0 < speed <= .1:
        raise ValueError('Arm joint speed must be finite, positive and within configured ROS limit 0.1 rad/s')
    if type(profile['scene']) is not dict:
        raise ValueError('Arm motion scene must be an object')
    return profile


def pose_specs():
    return {'side':{'type':'string','enum':['right']}, 'frame_id':{'type':'string','enum':list(FRAMES)},
            'tool_frame':{'type':'string'},
            **{k:{'type':'number','unit':'m'} for k in ('x_m','y_m','z_m')},
            **{k:{'type':'number'} for k in ('qx','qy','qz','qw')}}


class ArmMotionSkills(MotionRuntime):
    def __init__(self, profile, backend, *, directory='log/hei_arm', urdf=None):
        super().__init__(profile,backend,directory=directory,urdf=urdf,
                         worker_script=Path(__file__).with_name('worker.py'))
        self.fk = RightArmFK(self.urdf)

    def state(self, *, at_rest=True):
        return self.backend.arm_scene_state(at_rest=at_rest)

    def register(self, skills):
        skills._config['arm_motion_binding'] = self.binding
        identity = {'side':{'type':'string','enum':['right']},
                    'frame_id':{'type':'string','enum':list(FRAMES)}}
        skills.register('get_arm_pose',
            'Read fresh right-arm a_right_tcp pose by URDF FK. Explicit frame, meters and unit xyzw quaternion. '
            'No motion; position feedback required. Does not use a camera or detector.',
            identity,pose_specs(),self.get_pose)
        args = {**identity, **{k:{'type':'number','unit':'m'} for k in ('x_m','y_m','z_m')},
                'path_mode':{'type':'string','enum':['joint','linear']}}
        outputs = {**pose_specs(),'plan_id':{'type':'string'},'validated':{'type':'boolean'},
                   'validation_status':{'type':'string'},'plan_ref':{'type':'string'},
                   'nominal_duration_sec':{'type':'number','unit':'s'}}
        description = ('Plan a right-arm TCP target in an explicit frame. No motion. '
                       'joint means a quintic joint-space segment, NOT straight TCP motion; linear samples '
                       'a straight TCP path with rotation interpolation. IK and sampled full-robot/table collisions '
                       'must pass; no obstacle detour planning. Plan is single-use, expires in 60s. '
                       'No LLM-invented target coordinates; use user-specified values or observed pose outputs. ')
        skills.register('plan_arm_position', description+'Preserve the current TCP orientation; XYZ in meters.',
                        args,outputs,self.plan,timeout=90)
        skills.register('plan_arm_pose', description+'Position in meters plus explicit unit quaternion qx,qy,qz,qw.',
                        {**args, **{k:{'type':'number'} for k in ('qx','qy','qz','qw')}},
                        outputs,self.plan,timeout=90)
        skills.register('execute_arm_plan',
            'Execute one validated, unexpired arm plan via FollowJointTrajectory. Keeps the gripper unchanged; '
            'no opening/closing/automatic return. Requires matching start pose and scene, monitors motion of '
            'base/lift/left arm/grippers, verifies measured TCP endpoint. No automatic retry.',
            {'plan_id':{'type':'string'}},
            {**pose_specs(), 'plan_id':{'type':'string'},'reached_goal':{'type':'boolean'},
             'position_error_m':{'type':'number','unit':'m'},'orientation_error_rad':{'type':'number','unit':'rad'}},
            self.execute,motion=True,timeout=300)

    def measured_pose(self, state, frame):
        return np.linalg.inv(base_from_frame(self.urdf,state,frame)) @ self.fk(state['right'])

    async def get_pose(self, arguments, invocation):
        if self._binding() != self.binding:
            raise RuntimeError('Arm model/configuration changed; restart and rediscover')
        state = self.backend.arm_pose_state()
        return pose_fields(self.measured_pose(state,arguments['frame_id']),arguments['frame_id'])

    async def plan(self, arguments, invocation):
        if self.computing:
            raise RuntimeError('An arm plan is already being computed')
        self.computing = True
        try:
            state = self.state()
            if self._binding() != self.binding:
                raise RuntimeError('Arm model/configuration changed; restart and rediscover')
            plan_id = uuid.uuid4().hex
            directory = self.directory/plan_id
            directory.mkdir(parents=True)
            request = {'directory':str(directory), 'profile':self.profile, 'state':state,
                       'arguments':arguments, 'urdf':self.urdf, 'binding':self.binding}
            (directory/'request.json').write_text(json.dumps(request,allow_nan=False),encoding='utf-8')
            data = await self._worker(directory)
            if not self.same_start(state,self.state()):
                raise RuntimeError('Robot moved while computing arm plan; replan')
            self.remember(plan_id,data,state)
            return {**data['plan']['target_pose'], 'plan_id':plan_id,
                    'validated':data['validation']['status']=='passed',
                    'validation_status':data['validation']['status'], 'plan_ref':str(directory/'result.json'),
                    'nominal_duration_sec':data['plan']['nominal_duration_sec']}
        finally:
            self.computing = False

    async def execute(self, arguments, invocation):
        plan_id = arguments['plan_id']
        entry = self.checked_plan(plan_id)
        state = self.state()
        if not self.same_start(entry['state'],state):
            raise RuntimeError('Robot pose/scene differs from arm plan; replan')
        entry['consumed'] = True
        plan = entry['data']['plan']
        async def run():
            await self.backend.arm_trajectory(invocation,state['right'],plan['waypoints_rad'],
                                              plan['joint_speed_rad_s'])
        matches = lambda after: (self.scene_matches(state,after) and
                                 abs(state['right_gripper']-after['right_gripper']) <= .02)
        await self.guarded(run,matches,message='Base, lift, left arm or gripper moved during arm motion')
        final = self.state()
        if not matches(final):
            raise RuntimeError('Arm scene changed before endpoint verification')
        actual = self.fk(final['right'])
        target = np.asarray(plan['T_goal'],dtype=float)
        position_error = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        rotation_error = float(np.linalg.norm(cv2.Rodrigues(target[:3,:3].T @ actual[:3,:3])[0]))
        if position_error > .005 or rotation_error > math.radians(3):
            raise RuntimeError(f'Measured TCP target not reached: position_error_m={position_error:.6f}, '
                               f'orientation_error_rad={rotation_error:.6f}')
        return {**pose_fields(self.measured_pose(final,plan['target_frame']),plan['target_frame']),
                'plan_id':plan_id,'reached_goal':True,
                'position_error_m':position_error,'orientation_error_rad':rotation_error}
