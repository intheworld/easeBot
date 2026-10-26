"""Robot-owned, single-use right grasp plans; external compute never owns hardware."""
from copy import deepcopy
from collections import OrderedDict
import json
import math
import os
import re
from pathlib import Path
import time
import uuid

import numpy as np
from ..motion import MotionRuntime, data_directory
from .common_utils import grasp_target


def prepare_config(overrides):
    if type(overrides) is not dict:
        raise ValueError('right_grasp must be an object of profile overrides')
    base = json.loads((data_directory() / 'right_grasp.json').read_text())
    if set(overrides) - set(base):
        raise ValueError('Unknown right_grasp profile settings')
    base.update(deepcopy(overrides))
    for key in ('worker_python', 'model_path'):
        base[key] = str(Path(base[key]).expanduser().resolve(strict=True))
    if not os.access(base['worker_python'], os.X_OK):
        raise ValueError('Grasp worker Python is not executable')
    limits = {'joint_speed_rad_s': (0.001, .1), 'gripper_open_rad': (-4.5, 0),
              'gripper_closed_rad': (-4.5, 0), 'gap_per_motor_rad_m': (-.1, -.0001),
              'pregrasp_offset_m': (.001, .3), 'topdown_grip_depth_m': (0, .03),
              'finger_clearance_m': (0, .02), 'jaw_clearance_m': (0, .02), 'min_base_z_m': (-2, 2)}
    for key, (low, high) in limits.items():
        v = base[key]
        if key == 'min_base_z_m' and v is None:
            continue
        if type(v) not in (int, float) or not math.isfinite(v) or not low <= v <= high:
            raise ValueError(f'Invalid grasp setting: {key}')
    if base['gripper_open_rad'] >= base['gripper_closed_rad']:
        raise ValueError('Grasp open angle must be less than closed angle')
    if type(base['scene']) is not dict:
        raise ValueError('Grasp scene must be an object')
    return base


class GraspSkills(MotionRuntime):
    def __init__(self, profile, cameras, backend, *, directory='log/hei_grasp', urdf=None):
        self.cameras = cameras
        self.objects = OrderedDict()
        self.calibration = str((data_directory()/'right_eye_to_hand.npz').resolve(strict=True))
        super().__init__(profile, backend, directory=directory, urdf=urdf,
                         worker_script=Path(__file__).with_name('worker.py'),
                         extra_files=(self.calibration, profile['model_path']),
                         worker_cwd=Path(profile['model_path']).parent.parent)

    def register(self, skills):
        plan_outputs = {'plan_id': {'type': 'string'}, 'target': {'type': 'string'}, 'validation_status': {'type': 'string'},
             'plan_ref': {'type': 'string'}, 'validated': {'type': 'boolean'}, 'frame_id': {'type': 'string'},
             'x_m': {'type': 'number', 'unit': 'm'}, 'y_m': {'type': 'number', 'unit': 'm'},
             'z_m': {'type': 'number', 'unit': 'm'}, 'simulated': {'type': 'boolean'}}
        # Bind calibration/model/profile as well as skill signatures to the catalog revision.
        skills._config['right_grasp_binding'] = self.binding
        skills.register('perceive_grasp_target',
            'Perceive an open-vocabulary object from a real front RGB-D observation. No motion. '
            'Return an immutable object_id and a gravity-aligned 3D OBB of VISIBLE SURFACES in '
            'a_right_base_link; hidden geometry is unknown. Object expires 60s after capture. '
            'Use object_id with plan_right_grasp_from_object.',
            {'observation_id':{'type':'string'},'target':{'type':'string'}},
            {'object_id':{'type':'string'},'obb_ref':{'type':'string'},'target':{'type':'string'},
             'frame_id':{'type':'string'},'geometry_scope':{'type':'string'},
             'confidence':{'type':'number'},'point_count':{'type':'number'},
             **{k:{'type':'number','unit':'m'} for k in ('x_m','y_m','z_m','size_x_m','size_y_m','size_z_m')}},
            self.perceive,timeout=90)
        skills.register('plan_right_grasp_from_object',
            'Plan a right-arm top-down direct-IK grasp from a recent perceived object_id. No motion. '
            'Try short-axis and symmetric OBB candidates that fit the gripper; align above the '
            'object BEFORE descending with a fixed orientation. Use URDF finger envelope and '
            'configured table, then sample collision checks. Execute only validated=true. '
            'The resulting execute_right_grasp stops after closure; no automatic lift or return. '
            'No object-held verification or hidden-object geometry inference.',
            {'object_id':{'type':'string'}},plan_outputs,self.plan_object,timeout=180)
        skills.register('execute_right_grasp',
            'Execute one unexpired validated right grasp plan via ROS actions: open, pregrasp, approach, '
            'current-limited close, then STOP at closure without lift or return. '
            'Plan is single-use; arm/scene must match. '
            'Completion or stalled closure does not verify object capture. No automatic retry.',
            {'plan_id': {'type': 'string'}}, {'plan_id': {'type': 'string'}, 'completed': {'type': 'boolean'},
             'gripper_stalled': {'type': 'boolean'}, 'grasp_verified': {'type': 'boolean'}},
            self.execute, motion=True, timeout=300)

    def state(self, *, at_rest=True):
        return self.backend.grasp_state(at_rest=at_rest)

    def capture_request(self, arguments):
        target = grasp_target(arguments['target'])
        snapshot = self.cameras._get(arguments['observation_id'])
        if (snapshot['simulated'] or snapshot['camera'] != 'front' or snapshot['depth'] is None or
                snapshot['info'] is None or snapshot['info']['frame_id'] != 'front_camera_optical_frame'):
            raise RuntimeError('Grasp requires real front RGB-D with matching optical calibration')
        if time.monotonic()-snapshot['created'] > 3:
            raise RuntimeError('Capture a new front observation immediately before planning')
        state = self.backend.arm_scene_state() if hasattr(self.backend,'arm_scene_state') else self.state()
        if self._binding() != self.binding:
            raise RuntimeError('Grasp model/calibration/configuration changed; restart and rediscover')
        id = uuid.uuid4().hex
        directory = self.directory/id;directory.mkdir(parents=True)
        (directory/'rgb.jpg').write_bytes(snapshot['assets']['rgb'][1])
        np.save(directory/'depth.npy',snapshot['depth'],allow_pickle=False)
        request = {'directory':str(directory),'operation':'perceive','profile':self.profile,'state':state,
                   'info':snapshot['info'],'target':target,'calibration':self.calibration,'urdf':self.urdf,
                   'observation_id':arguments['observation_id'],'binding':self.binding}
        (directory/'request.json').write_text(json.dumps(request,allow_nan=False))
        return id,directory,request,state,snapshot['created']

    @staticmethod
    def plan_outputs(id,target,directory,data):
        xyz = data['plan']['T_grasp']
        return {'plan_id':id,'target':target,'validation_status':data['validation']['status'],
                'validated':data['validation']['status']=='passed','plan_ref':str(directory/'result.json'),
                'frame_id':'a_right_base_link','x_m':xyz[0][3],'y_m':xyz[1][3],'z_m':xyz[2][3],'simulated':False}

    async def perceive(self,arguments,invocation):
        if self.computing:raise RuntimeError('A grasp computation is already running')
        self.computing = True
        try:
            id,directory,request,state,created = self.capture_request(arguments)
            data = await self._worker(directory)
            after = self.backend.arm_scene_state() if hasattr(self.backend,'arm_scene_state') else self.state()
            if not self.scene_matches(state,after):raise RuntimeError('Scene moved during perception; recapture')
            if time.monotonic()-created > 60:raise RuntimeError('Object observation expired; recapture')
            obb = data['object']
            if obb.get('schema')!='hei.obb/v1' or obb.get('target')!=request['target']:
                raise RuntimeError('Invalid object computation result')
            while len(self.objects)>=8:self.objects.popitem(last=False)
            self.objects[id] = {'object':deepcopy(obb),'state':state,'created':created}
            return {'object_id':id,'obb_ref':str(directory/'result.json'),'target':obb['target'],
                    'frame_id':obb['frame_id'],'geometry_scope':obb['geometry_scope'],
                    'confidence':obb['confidence'],'point_count':obb['point_count'],
                    **dict(zip(('x_m','y_m','z_m'),obb['center_m'])),
                    **dict(zip(('size_x_m','size_y_m','size_z_m'),obb['size_m']))}
        finally:self.computing = False

    async def plan_object(self,arguments,invocation):
        if self.computing:raise RuntimeError('A grasp computation is already running')
        self.computing = True
        try:
            id = arguments['object_id']
            if not re.fullmatch('[0-9a-f]{32}',id):raise ValueError('Invalid object ID')
            entry = self.objects.get(id)
            if entry is None or time.monotonic()-entry['created']>60:
                raise RuntimeError('Object missing or expired; capture and perceive again')
            state = self.state()
            if not self.scene_matches(entry['state'],state):raise RuntimeError('Scene moved since perception; recapture')
            if self._binding()!=self.binding:raise RuntimeError('Model/configuration changed; restart and recapture')
            plan_id = uuid.uuid4().hex
            directory = self.directory/plan_id;directory.mkdir(parents=True)
            request = {'directory':str(directory),'operation':'plan_object','object':entry['object'],
                       'object_id':id,'state':state,'profile':self.profile,'urdf':self.urdf,'binding':self.binding}
            (directory/'request.json').write_text(json.dumps(request,allow_nan=False))
            data = await self._worker(directory)
            if not self.same_start(state,self.state()):raise RuntimeError('Robot moved during planning; recapture')
            if time.monotonic()-entry['created']>60:raise RuntimeError('Object expired while planning; recapture')
            self.remember(plan_id,data,state)
            return self.plan_outputs(plan_id,entry['object']['target'],directory,data)
        finally:self.computing = False

    async def execute(self, arguments, invocation):
        plan_id = arguments['plan_id']
        entry = self.checked_plan(plan_id)
        plan = entry['data']['plan']
        if plan.get('schema_version') != 3 or plan.get('strategy') != 'top_down_direct_ik':
            raise ValueError('Unsupported grasp plan; regenerate an OBB plan')
        state = self.state()
        if not self.same_start(entry['state'],state):
            raise RuntimeError('Robot pose differs from grasp plan; replan')
        await self.backend.grasp_check(self.profile)
        entry['consumed'] = True  # Never replay after partial execution or cancellation.
        async def run():
            await self.backend.grasp_gripper(invocation, -self.profile['gripper_open_rad'] * abs(self.profile['gap_per_motor_rad_m']), close=False)
            await self.backend.grasp_trajectory(invocation, state['right'], [plan['pregrasp_joint_rad']], self.profile['joint_speed_rad_s'])
            await self.backend.grasp_trajectory(invocation, plan['pregrasp_joint_rad'], plan['approach_joint_rad'], self.profile['joint_speed_rad_s'])
            stalled = await self.backend.grasp_gripper(invocation, -self.profile['gripper_closed_rad'] * abs(self.profile['gap_per_motor_rad_m']), close=True)
            return stalled

        stalled = await self.guarded(run, lambda after: self.scene_matches(state, after),
                                     message='Base, lift or left arm moved during grasp')
        return {'plan_id': plan_id, 'completed': True, 'gripper_stalled': stalled, 'grasp_verified': False}
