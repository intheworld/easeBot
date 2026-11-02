"""Robot-owned TCP pose skills. Planning never sends a ROS goal."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import uuid
from collections import OrderedDict

import cv2
import numpy as np

from ..grasp.kinematics import RightArmFK
from ..motion import MotionRuntime, data_directory
from .transport import TransportSkills
from .regions import PlacementRegions
from .geometry import FRAMES, base_from_frame, pose_fields


def prepare_config(overrides):
    if type(overrides) is not dict:
        raise ValueError('arm_motion must be an object of profile overrides')
    profile = json.loads((data_directory()/'arm_motion.json').read_text())
    if set(overrides)-set(profile):
        raise ValueError('Unknown arm_motion settings')
    profile.update(deepcopy(overrides))
    if profile.get('ik_backend','auto') not in ('auto','scipy','curobo'):
        raise ValueError('Invalid ik_backend')
    profile['worker_python'] = str(Path(profile['worker_python']).expanduser().absolute())
    if not os.access(profile['worker_python'],os.X_OK):
        raise ValueError('Motion worker Python is not executable')
    speed = profile['joint_speed_rad_s']
    if type(speed) not in (int,float) or not math.isfinite(speed) or not 0 < speed <= 1.:
        raise ValueError('Arm joint speed must be finite, positive and within trajectory limit 1 rad/s')
    for key,low,high in [('gap_per_motor_rad_m',-.1,-.0001),('release_clearance_m',.001,.03),('payload_padding_m',0.,.03)]:
        v=profile[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high:
            raise ValueError(f'Invalid transport setting: {key}')
    if profile['release_clearance_m']<=profile['payload_padding_m']:
        raise ValueError('Release clearance must exceed payload padding')
    from .spatial import unknown_fraction
    unknown_fraction(profile)
    if type(profile['scene']) is not dict:
        raise ValueError('Arm motion scene must be an object')
    from ..motion import validate_compute_config
    validate_compute_config(profile)
    return profile


def pose_specs():
    return {'side':{'type':'string','enum':['right']}, 'frame_id':{'type':'string','enum':list(FRAMES)},
            'tool_frame':{'type':'string'},
            **{k:{'type':'number','unit':'m'} for k in ('x_m','y_m','z_m')},
            **{k:{'type':'number'} for k in ('qx','qy','qz','qw')}}


class ArmMotionSkills(PlacementRegions,TransportSkills,MotionRuntime):
    def __init__(self, profile, backend, *, directory='log/hei_arm', urdf=None, carry=None, region_perception=None):
        from .payload import CarryState
        self.carry=carry if carry is not None else CarryState()
        if self.carry.gap_per_rad is None:
            self.carry.configure_gripper(-4.5,profile['gap_per_motor_rad_m'])
        self.region_perception=region_perception
        self.paths=OrderedDict()
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
        self.register_transport(skills,outputs)
        self.register_regions(skills,outputs)
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
        skills.register('plan_arm_home',
            'Plan right-arm return to six joint angles of 0 rad. No motion and no encoder zeroing. '
            'Keeps the gripper unchanged. Joint-space trajectory must pass joint-limit and sampled '
            'robot/table collision checks; no obstacle detour. Execute only when validated=true.',
            {'side':{'type':'string','enum':['right']}},outputs,self.plan_home,timeout=90)
        skills.register('execute_arm_plan',
            'Execute one validated, unexpired arm plan via FollowJointTrajectory. Keeps the gripper unchanged; '
            'no opening/closing/automatic return. Requires matching start pose and scene, monitors motion of '
            'base/lift/right gripper, verifies measured TCP endpoint. No automatic retry.',
            {'plan_id':{'type':'string'}},
            {**pose_specs(), 'plan_id':{'type':'string'},'reached_goal':{'type':'boolean'},
             'position_error_m':{'type':'number','unit':'m'},'orientation_error_rad':{'type':'number','unit':'rad'},
             'joint_error_rad':{'type':'number','unit':'rad'}},
            self.execute,motion=True,timeout=300)

    def measured_pose(self, state, frame):
        return np.linalg.inv(base_from_frame(self.urdf,state,frame)) @ self.fk(state['right'])

    async def get_pose(self, arguments, invocation):
        if self._binding() != self.binding:
            raise RuntimeError('Arm model/configuration changed; restart and rediscover')
        state = self.backend.arm_pose_state()
        return pose_fields(self.measured_pose(state,arguments['frame_id']),arguments['frame_id'])

    async def plan_home(self, arguments, invocation):
        return await self.plan(dict(side=arguments['side'],frame_id='a_right_base_link',
                                    path_mode='joint',joint_home=True),invocation)

    async def plan(self, arguments, invocation):
        if self.computing:
            raise RuntimeError('An arm plan is already being computed')
        self.computing = True
        try:
            state = self.state()
            carry=self.carry.check(state);carry_token=self.carry.token
            if self._binding() != self.binding:
                raise RuntimeError('Arm model/configuration changed; restart and rediscover')
            plan_id = uuid.uuid4().hex
            directory = self.directory/plan_id
            directory.mkdir(parents=True)
            request = {'directory':str(directory), 'profile':self.profile, 'state':state,
                       'arguments':arguments, 'urdf':self.urdf, 'binding':self.binding,'carry':carry}
            (directory/'request.json').write_text(json.dumps(request,allow_nan=False),encoding='utf-8')
            data = await self._worker(directory)
            if not self.same_start(state,self.state()):
                raise RuntimeError('Robot moved while computing arm plan; replan')
            if self.carry.token!=carry_token:raise RuntimeError('Carried object scene changed during planning')
            self.remember(plan_id,data,state)
            self.plans[plan_id]['carry_token']=carry_token
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
        self.carry.check(state)
        if self.carry.token!=entry['carry_token']:raise RuntimeError('Carried object scene changed; replan')
        if entry['data']['plan'].get('operation')=='place':raise ValueError('Use execute_place_plan for placement')
        if not self.same_start(entry['state'],state):
            raise RuntimeError('Robot pose/scene differs from arm plan; replan')
        entry['consumed'] = True
        plan = entry['data']['plan']
        async def run():
            await self.backend.arm_trajectory(invocation,state['right'],plan['waypoints_rad'],
                                              plan['joint_speed_rad_s'])
        matches = lambda after: (self.scene_matches(state,after) and
                                 abs(state['right_gripper']-after['right_gripper']) <= .02)
        await self.guarded(run,matches,message='Base, lift or right gripper moved during arm motion')
        final = self.state()
        if not matches(final):
            raise RuntimeError('Arm scene changed before endpoint verification')
        position_error,rotation_error,joint_error=self.verify_endpoint(plan,final)
        return {**pose_fields(self.measured_pose(final,plan['target_frame']),plan['target_frame']),
                'plan_id':plan_id,'reached_goal':True,
                'position_error_m':position_error,'orientation_error_rad':rotation_error,'joint_error_rad':joint_error}


    def verify_endpoint(self,plan,final):
        actual = self.fk(final['right'])
        target = np.asarray(plan['T_goal'],dtype=float)
        position_error = float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        rotation_error = float(np.linalg.norm(cv2.Rodrigues(target[:3,:3].T @ actual[:3,:3])[0]))
        joint_error = float(np.max(np.abs(np.asarray(final['right'])-np.asarray(plan['waypoints_rad'][-1]))))
        if position_error > .005 or rotation_error > math.radians(3):
            raise RuntimeError(f'Measured TCP target not reached: position_error_m={position_error:.6f}, '
                               f'orientation_error_rad={rotation_error:.6f}')
        if joint_error > .03:
            raise RuntimeError(f'Measured joint target not reached: joint_error_rad={joint_error:.6f}')
        return position_error,rotation_error,joint_error
