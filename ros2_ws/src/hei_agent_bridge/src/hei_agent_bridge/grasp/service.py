"""Robot-owned, single-use right grasp plans; external compute never owns hardware."""
from copy import deepcopy
import asyncio
from collections import OrderedDict
import json
import math
import os
import re
from pathlib import Path
import time
import uuid

import numpy as np
from ..motion import MotionRuntime, MotionComputeError, data_directory
from .common_utils import grasp_target
from .wrist import refinement_config, load_calibration


def prepare_config(overrides):
    if type(overrides) is not dict:
        raise ValueError('right_grasp must be an object of profile overrides')
    base = json.loads((data_directory() / 'right_grasp.json').read_text())
    base.setdefault('detector_prompt_aliases', {})
    if set(overrides) - set(base):
        raise ValueError('Unknown right_grasp profile settings')
    base.update(deepcopy(overrides))
    if base.get('ik_backend','auto') not in ('auto','scipy','curobo'):
        raise ValueError('Invalid ik_backend')
    for key in ('worker_python', 'model_path'):
        path = Path(base[key]).expanduser()
        base[key] = str(path.absolute() if key == 'worker_python' else path.resolve(strict=True))
    if not os.access(base['worker_python'], os.X_OK):
        raise ValueError('Grasp worker Python is not executable')
    limits = {'joint_speed_rad_s': (0.001, 1.), 'gripper_open_rad': (-4.5, 0),
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
    from .detection import prompt_aliases
    base['detector_prompt_aliases'] = prompt_aliases(base['detector_prompt_aliases'])
    base['wrist_refinement'] = refinement_config(base['wrist_refinement'])
    from ..motion import validate_compute_config
    validate_compute_config(base)
    return base


class GraspSkills(MotionRuntime):
    def __init__(self, profile, cameras, backend, *, directory='log/hei_grasp', urdf=None, carry=None):
        from ..arm.payload import CarryState
        self.carry=carry if carry is not None else CarryState()
        profile=deepcopy(profile)
        profile['wrist_refinement']=refinement_config(profile.get('wrist_refinement',{}))
        wrist_file=profile['wrist_refinement']['calibration_file']
        if wrist_file:
            wrist_file=str(Path(wrist_file).expanduser().resolve(strict=True))
            profile['wrist_refinement']['calibration_file']=wrist_file
        self.carry.configure_gripper(profile['gripper_open_rad'],profile['gap_per_motor_rad_m'])
        self.cameras = cameras
        self.objects = OrderedDict()
        self.calibration = str((data_directory()/'right_eye_to_hand.npz').resolve(strict=True))
        super().__init__(profile, backend, directory=directory, urdf=urdf,
                         worker_script=Path(__file__).with_name('worker.py'),
                         extra_files=(self.calibration, profile['model_path'], *((wrist_file,) if wrist_file else ())),
                         worker_cwd=Path(profile['model_path']).parent.parent)

    def register(self, skills):
        plan_outputs = {'plan_id': {'type': 'string'}, 'target': {'type': 'string'}, 'validation_status': {'type': 'string'},
             'plan_ref': {'type': 'string'}, 'validated': {'type': 'boolean'}, 'frame_id': {'type': 'string'},
             'x_m': {'type': 'number', 'unit': 'm'}, 'y_m': {'type': 'number', 'unit': 'm'},
             'z_m': {'type': 'number', 'unit': 'm'}, 'simulated': {'type': 'boolean'}}
        # Bind calibration/model/profile as well as skill signatures to the catalog revision.
        skills._config['right_grasp_binding'] = self.binding
        skills.register('observe_gripper',
            'Immediately read fresh right-gripper motor opening and sustained measured-load contact. '
            'No camera, image capture, visual inference, arm/base/lift settling or motion command. '
            'physical_capture_verified requires a completed close, enabled stable jaw above closing target '
            'and measured motor torque >=0.1 Nm for >=0.25s. Missing/stale feedback is unverified. '
            'Uses the already collected feedback window, does not wait to collect a new one. '
            'Uniform grasp contact evidence, not a holding-force rating or slip guarantee. '
            'wrist_state=not_requested; use observe_gripper_visual only for optional RGB inspection.',
            {'side':{'type':'string','enum':['right']}},
            {'holding_state':{'type':'string'},'jaw_state':{'type':'string'},'gap_m':{'type':'number','unit':'m'},
             'wrist_state':{'type':'string'},'object_visible':{'type':'boolean'},
             'observation_id':{'type':'string'},'model_present':{'type':'boolean'},
             'physical_capture_verified':{'type':'boolean'},'contact_state':{'type':'string'},
             'torque_nm':{'type':'number','unit':'Nm'}},self.observe_gripper,motion=True,timeout=60)
        skills.register('observe_gripper_visual',
            'Read fresh right gripper opening and wrist RGB. For current-session payload geometry, '
            'check whether the known target is visually consistent with its expected position. '
            'Missing/occluded/ambiguous RGB is inconclusive, never proof of empty or firmly held. '
            'physical_capture_verified reports sustained contact after completed closure: gap above closing target, stable enabled jaw and measured motor torque >=0.1 Nm for >=0.25s. '
            'This is jaw/load contact evidence, not a force rating or slip guarantee. Missing/stale feedback returns false. '
            'No movement or gripper command. Does not create/restore payload geometry.',
            {'side':{'type':'string','enum':['right']}},
            {'holding_state':{'type':'string'},'jaw_state':{'type':'string'},'gap_m':{'type':'number','unit':'m'},
             'wrist_state':{'type':'string'},'object_visible':{'type':'boolean'},
             'observation_id':{'type':'string'},'model_present':{'type':'boolean'},
             'physical_capture_verified':{'type':'boolean'},'contact_state':{'type':'string'},
             'torque_nm':{'type':'number','unit':'Nm'}},self.observe_gripper_visual,motion=True,timeout=60)
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
            'When wrist_refinement is enabled, require measured wrist calibration before motion; '
            'at hover use fresh wrist RGB and bounded planar corrections with collision checks, '
            'then reobserve before descent. Missing/ambiguous target or nonconvergence aborts. '
            'Plan is single-use; arm/scene must match. '
            'Completion or stalled closure does not verify object capture. No automatic retry.',
            {'plan_id': {'type': 'string'}}, {'plan_id': {'type': 'string'}, 'completed': {'type': 'boolean'},
             'gripper_stalled': {'type': 'boolean'}, 'grasp_verified': {'type': 'boolean'},
             'wrist_refined': {'type': 'boolean'}},
            self.execute, motion=True, timeout=300)

    async def observe_gripper(self,arguments,invocation):
        position=getattr(self.backend,'gripper_position',None)
        state={'right_gripper':position()} if position is not None else self.state()
        self.carry.observe(state)
        observed=self.carry.observation
        output=dict(holding_state='open' if observed['jaw_state']=='open' else 'unknown',
            jaw_state=observed['jaw_state'],gap_m=observed['gap_m'],wrist_state='not_requested',
            object_visible=False,observation_id='',model_present=self.carry.payload is not None,
            physical_capture_verified=False,contact_state='feedback_unavailable',torque_nm=0.)
        contact=getattr(self.backend,'gripper_contact',None)
        if contact is not None:output.update(contact(self.profile))
        self.carry.apply_contact(output)
        output['model_present']=self.carry.payload is not None
        if output['physical_capture_verified']:output['holding_state']='holding_contact_confirmed'
        return output

    async def observe_gripper_visual(self,arguments,invocation):
        if self.computing:raise RuntimeError('A perception computation is already running')
        self.computing=True
        try:
            state=self.state();self.carry.observe(state)
            payload=deepcopy(self.carry.payload);token=self.carry.token
            output=dict(holding_state='open' if self.carry.observation['jaw_state']=='open' else 'unknown',
                jaw_state=self.carry.observation['jaw_state'],gap_m=self.carry.observation['gap_m'],
                wrist_state='no_current_model' if payload is None else 'uncalibrated',
                object_visible=False,observation_id='',model_present=payload is not None,
                physical_capture_verified=False,contact_state='feedback_unavailable',torque_nm=0.)
            contact=getattr(self.backend,'gripper_contact',None)
            if contact is not None:output.update(contact(self.profile))
            if output['physical_capture_verified']:output['holding_state']='holding_contact_confirmed'
            config=self.profile['wrist_refinement']
            if payload is None or not config['calibration_file']:
                try:capture=await self.cameras.capture({'camera':'right_wrist'},invocation)
                except RuntimeError:
                    output['wrist_state']='camera_unavailable'
                    return output
                output['observation_id']=capture['observation_id']
                return output
            directory=self.directory/('holding_'+uuid.uuid4().hex);directory.mkdir(parents=True)
            from .capture import acquire
            capture,image,matched=await acquire(self,directory,invocation,config,0,[])
            output['observation_id']=capture['observation_id']
            request=dict(operation='observe_payload',directory=str(directory),profile=self.profile,
                payload=payload,urdf=self.urdf,state=matched,image_stamp_sec=image['stamp_sec'])
            (directory/'request.json').write_text(json.dumps(request,allow_nan=False))
            data=await asyncio.wait_for(self._worker(directory),config['compute_timeout_sec'])
            after=self.state()
            joint_delta=max(abs(a-b) for a,b in zip(matched['right'],after['right']))
            if (not self.same_start(matched,after) or joint_delta>config['joint_span_rad']
                    or token!=self.carry.token):
                output.update(physical_capture_verified=False,contact_state='scene_changed',holding_state='unknown')
                output['wrist_state']='scene_changed'
                return output
            output.update(data)
            if contact is not None:output.update(contact(self.profile))
            if output['object_visible'] and output['jaw_state']!='open':
                output['holding_state']='holding_candidate'
            if output['physical_capture_verified']:output['holding_state']='holding_contact_confirmed'
            self.carry.observation['wrist_state']=output['wrist_state']
            return output
        finally:
            if 'output' in locals() and output['contact_state']!='scene_changed' and contact is not None:
                output.update(contact(self.profile))
                if output['physical_capture_verified']:output['holding_state']='holding_contact_confirmed'
                elif output['holding_state']=='holding_contact_confirmed':
                    output['holding_state']='holding_candidate' if output['object_visible'] else 'unknown'
            self.computing=False

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
            self.carry.observe(state)
            self.carry.require_empty()
            carry=self.carry.check(state);carry_token=self.carry.token
            if not self.scene_matches(entry['state'],state):raise RuntimeError('Scene moved since perception; recapture')
            if self._binding()!=self.binding:raise RuntimeError('Model/configuration changed; restart and recapture')
            plan_id = uuid.uuid4().hex
            directory = self.directory/plan_id;directory.mkdir(parents=True)
            # Associate a fresh observation with at most one remembered placed
            # box. Only finger contact is permitted until closure succeeds.
            from ..arm.geometry import base_from_frame
            observed=deepcopy(entry['object']);matches=[]
            for obstacle in carry['obstacles']:
                if obstacle.get('kind')=='place_region':continue
                t=base_from_frame(self.urdf,state,'base_footprint')@np.asarray(obstacle['T_footprint_object'])
                local=t[:3,:3].T@(np.asarray(observed['center_m'])-t[:3,3])
                if np.all(np.abs(local)<=np.asarray(obstacle['size_m'])/2+.01):matches.append(obstacle['id'])
            if len(matches)>1:raise RuntimeError('Observed target matches multiple placed objects; reconcile scene')
            if matches:observed['known_obstacle_id']=matches[0]
            request = {'directory':str(directory),'operation':'plan_object','object':observed,
                       'object_id':id,'state':state,'carry':carry,'profile':self.profile,'urdf':self.urdf,'binding':self.binding}
            (directory/'request.json').write_text(json.dumps(request,allow_nan=False))
            data = await self._worker(directory)
            if not self.same_start(state,self.state()):raise RuntimeError('Robot moved during planning; recapture')
            if time.monotonic()-entry['created']>60:raise RuntimeError('Object expired while planning; recapture')
            if self.carry.token!=carry_token:raise RuntimeError('Carried object scene changed during planning')
            self.remember(plan_id,data,state)
            self.plans[plan_id]['carry_token']=carry_token
            return self.plan_outputs(plan_id,entry['object']['target'],directory,data)
        finally:self.computing = False

    async def execute(self, arguments, invocation):
        plan_id = arguments['plan_id']
        entry = self.checked_plan(plan_id)
        plan = entry['data']['plan']
        if plan.get('schema_version') != 3 or plan.get('strategy') != 'top_down_direct_ik':
            raise ValueError('Unsupported grasp plan; regenerate an OBB plan')
        self.carry.observe(self.state())
        self.carry.require_empty()
        if entry.get('carry_token')!=self.carry.token:raise RuntimeError('Carried object scene changed; replan')
        state = self.state()
        if not self.same_start(entry['state'],state):
            raise RuntimeError('Robot pose differs from grasp plan; replan')
        wrist=self.profile['wrist_refinement']
        if wrist['enabled']:
            calibration=load_calibration(wrist['calibration_file'])
            frame=self.cameras.cache.frame('right_wrist')
            if (frame is None or frame.frame_id!=calibration['frame_id'] or
                    (frame.width,frame.height)!=(calibration['width'],calibration['height'])):
                raise RuntimeError('A fresh real right-wrist image matching measured calibration is required')
        await self.backend.grasp_check(self.profile)
        entry['consumed'] = True  # Never replay after partial execution or cancellation.
        async def run():
            await self.backend.grasp_gripper(invocation, -self.profile['gripper_open_rad'] * abs(self.profile['gap_per_motor_rad_m']), close=False)
            await self.backend.grasp_trajectory(invocation, state['right'], [plan['pregrasp_joint_rad']], self.profile['joint_speed_rad_s'])
            approach_plan=await self.refine_wrist(plan_id,plan,invocation) if wrist['enabled'] else plan
            await self.backend.grasp_trajectory(invocation, approach_plan['pregrasp_joint_rad'], approach_plan['approach_joint_rad'], self.profile['joint_speed_rad_s'])
            from .kinematics import RightArmFK
            before_close=self.state()
            self.carry.attach(approach_plan['target_obb'],RightArmFK(self.urdf)(before_close['right']),before_close['right_gripper'],pending=True)
            stalled = await self.backend.grasp_gripper(invocation, -self.profile['gripper_closed_rad'] * abs(self.profile['gap_per_motor_rad_m']), close=True)
            from .kinematics import RightArmFK
            actual=self.state()
            self.carry.attach(approach_plan['target_obb'],RightArmFK(self.urdf)(actual['right']),actual['right_gripper'],pending=True)
            contact=getattr(self.backend,'gripper_contact',None)
            evidence=contact(self.profile) if contact is not None else {}
            self.carry.apply_contact(evidence)
            return stalled,bool(evidence.get('physical_capture_verified',False))

        stalled,verified = await self.guarded(run, lambda after: self.scene_matches(state, after),
                                     message='Base or lift moved during grasp')
        return {'plan_id': plan_id, 'completed': True, 'gripper_stalled': stalled,
                'grasp_verified': verified, 'wrist_refined':wrist['enabled']}

    async def refine_wrist(self,plan_id,origin,invocation):
        config=self.profile['wrist_refinement']
        reference=deepcopy(origin)
        # max_updates movements, followed by one last observation to verify.
        for iteration in range(config['max_updates']+1):
            await asyncio.sleep(config['settle_sec'])
            directory=self.directory/plan_id/f'wrist_{iteration:02d}';directory.mkdir(parents=True)
            rows=[]
            from .capture import acquire,CaptureRetry,delta,record,scene_check
            for attempt in range(config['capture_retries']+1):
                try:
                    captured,image,state=await acquire(self,directory,invocation,config,attempt,rows)
                    request={'operation':'refine_wrist','directory':str(directory),'urdf':self.urdf,
                             'profile':self.profile,'state':state,'origin_plan':origin,'reference_plan':reference,
                             'binding':self.binding,'observation_id':captured['observation_id'],
                             'image_stamp_sec':image['stamp_sec']}
                    encoded=json.dumps(request,allow_nan=False)
                    (directory/'request.json').write_text(encoded)
                    (directory/f'request_attempt_{attempt:02d}.json').write_text(encoded)
                    (directory/f'rgb_attempt_{attempt:02d}.jpg').write_bytes(image['assets']['rgb'][1])
                    if self._binding()!=self.binding:raise RuntimeError('Wrist model/calibration changed; restart and replan')
                    try:
                        data=await asyncio.wait_for(self._worker(directory),config['compute_timeout_sec'])
                    except MotionComputeError as error:
                        if error.error_code=='wrist_observation_unavailable':raise CaptureRetry(str(error)) from error
                        raise
                    after_state=self.state(at_rest=False)
                    scene_check(self,state,after_state)
                    changes=delta(state,after_state)
                    record(directory,rows,'computed',attempt=attempt,state=after_state,delta=changes)
                    if changes['max_right_delta_rad']>config['joint_span_rad']:
                        raise CaptureRetry('Right arm drifted while computing wrist correction: '+json.dumps(changes))
                    if self._binding()!=self.binding:raise RuntimeError('Wrist model/calibration changed during refinement')
                    break
                except CaptureRetry as error:
                    record(directory,rows,'retry',attempt=attempt,reason=str(error))
                    if attempt==config['capture_retries']:
                        raise RuntimeError(f'Wrist observation remained unstable/unavailable: {error}; inspect {directory}/stability.json') from error
            if data['validation']['status']!='passed':
                raise RuntimeError(f'Wrist correction collision/scene validation failed; inspect {directory}')
            diagnostic_file=directory/'wrist.json'
            diagnostic=json.loads(diagnostic_file.read_text()) if diagnostic_file.exists() else {}
            error=diagnostic.get('alignment_error_m')
            remaining_m=float(np.linalg.norm(error)) if error is not None else None
            exhausted=iteration==config['max_updates'] and not data['converged']
            status=dict(converged=data['converged'],commands_executed=iteration,
                max_updates=config['max_updates'],remaining_error_m=remaining_m,
                tolerance_m=config['tolerance_m'],budget_exhausted=exhausted,
                next_correction_planned=not data['converged'] and not exhausted)
            (directory/'refinement_status.json').write_text(json.dumps(status,allow_nan=False))
            if data['converged']:return data['plan']
            if exhausted:
                remaining=f'{remaining_m*1000:.3f} mm' if remaining_m is not None else 'unavailable'
                raise RuntimeError(f'Wrist refinement did not converge after {iteration} corrections; '
                    f'remaining error {remaining}, tolerance {config["tolerance_m"]*1000:.3f} mm; '
                    f'no descent or closure executed; inspect {directory}/refinement_status.json')
            path=data['plan']['pregrasp_joint_path']
            await self.backend.grasp_trajectory(invocation,state['right'],path,self.profile['joint_speed_rad_s'])
            reference=data['reference_plan']
