"""Flat MHS transport skills with robot-owned paths and payload state."""
from copy import deepcopy
import json
import time
import uuid

import numpy as np
from .geometry import FRAMES,base_from_frame,quaternion_matrix


class TransportSkills:
    def register_transport(self,skills,plan_outputs):
        side={'side':{'type':'string','enum':['right']}}
        meters=lambda **limits:dict(type='number',unit='m',**limits)
        status={'status':{'type':'string'},'payload_id':{'type':'string'},'revision':{'type':'string'},
                'placed_count':{'type':'number'},'reason':{'type':'string'},'physical_capture_verified':{'type':'boolean'}}
        skills.register('get_carried_object','Read logical carried-object state, not proof of capture.',{},status,self.carry_status)
        for name,method,flag,description in [
            ('confirm_gripper_empty',self.confirm_empty,'observed_empty','Operator-only confirmation that gripper is physically empty. No motion.'),
            ('confirm_carried_object',self.confirm_attached,'observed_attached','Operator-only recovery of current-session geometry after checking object is still firmly held. No motion.'),
            ('clear_placed_objects',self.clear_placed,'scene_cleared','Operator-only confirmation that all remembered placed objects have been removed. No motion.')]:
            skills.register(name,description+' Never infer this confirmation from task text.',
                            {**side,flag:{'type':'boolean','enum':[True]}},status,method,motion=True)
        skills.register('set_carried_object_geometry',
            'Operator-specified complete box geometry in a_right_tcp: center meters, unit xyzw orientation and sizes. '
            'Requires explicit observed_attached=true and geometry_confirmed=true from the operator. Can declare an already-held object. Do not invent hidden dimensions. '
            'Invalidates previous plans; no motion.',
            {**side,'geometry_confirmed':{'type':'boolean','enum':[True]},'observed_attached':{'type':'boolean','enum':[True]},
             **{k:meters() for k in ('x_m','y_m','z_m')},
             **{k:meters(minimum=.0001) for k in ('size_x_m','size_y_m','size_z_m')},
             **{k:{'type':'number'} for k in ('qx','qy','qz','qw')}},status,self.set_geometry,motion=True)
        skills.register('plan_arm_offset','Plan a relative straight TCP displacement, preserving orientation. '
                        'frame_id determines displacement axes; a_right_tcp uses current tool axes. No motion.',
                        {**side,'frame_id':{'type':'string','enum':[*FRAMES,'a_right_tcp']},
                         **{k:meters() for k in ('dx_m','dy_m','dz_m')}},plan_outputs,self.plan_offset,timeout=90)
        path_result={'path_id':{'type':'string'},'waypoint_count':{'type':'number'}}
        skills.register('create_arm_path','Create a robot-owned empty TCP path, expires after 300s, at most 32 waypoints. No motion.',
                        {**side,'frame_id':{'type':'string','enum':list(FRAMES)},
                         'path_mode':{'type':'string','enum':['joint','linear']}},path_result,self.create_path)
        skills.register('append_arm_waypoint','Append a user-specified full TCP pose (meters, unit xyzw) to a path. No motion.',
                        {'path_id':{'type':'string'},**{k:meters() for k in ('x_m','y_m','z_m')},
                         **{k:{'type':'number'} for k in ('qx','qy','qz','qw')}},path_result,self.append_waypoint)
        skills.register('plan_arm_path','Freeze and validate the complete path, including every segment and carried-object collisions. '
                        'No motion, no automatic obstacle detours. Execute only validated=true.',
                        {'path_id':{'type':'string'}},plan_outputs,self.plan_path,timeout=90)
        skills.register('plan_place_object','Plan lift, transfer, descend, release and vertical retreat for the currently carried object. '
                        'x_m/y_m locate OBJECT CENTER above a horizontal support plane at support_z_m in frame_id. '
                        'Preserve current orientation. All stages checked; no motion. Never invent placement coordinates.',
                        {**side,'frame_id':{'type':'string','enum':list(FRAMES)},
                         **{k:meters() for k in ('x_m','y_m','support_z_m')},
                         'approach_height_m':meters(minimum=.01,maximum=.3),
                         'retreat_height_m':meters(minimum=.01,maximum=.3),
                         'release_gap_m':meters(minimum=.001,maximum=.1)},
                        plan_outputs,self.plan_place,timeout=180)
        skills.register('execute_place_plan','Execute a validated single-use place plan. Do not release if transfer fails. '
                        'After opening, remember placed geometry and revalidate retreat from fresh feedback. '
                        'Failure/cancel never automatically opens, retries or returns home. Completion is not proof of placement.',
                        {'plan_id':{'type':'string'}},
                        {'placed':{'type':'boolean'},'retracted':{'type':'boolean'},'placement_verified':{'type':'boolean'},
                         'payload_id':{'type':'string'},'release_started':{'type':'boolean'},
                         'release_confirmed':{'type':'boolean'},'failed_stage':{'type':'string'},
                         'detail_ref':{'type':'string'}},self.execute_place,motion=True,timeout=600)

    def _status(self):
        value=self.carry.snapshot()
        return dict(status=value['status'],payload_id=(value['payload'] or {}).get('id',''),revision=self.carry.token,
                    placed_count=len(value['obstacles']),reason=value['reason'],physical_capture_verified=False)

    async def carry_status(self,args,invocation):
        self.carry.observe(self.state())
        return self._status()
    async def confirm_empty(self,args,invocation):
        self.state();self.carry.confirm_empty();return self._status()
    async def clear_placed(self,args,invocation):
        self.state();self.carry.clear_obstacles();return self._status()
    async def confirm_attached(self,args,invocation):
        state=self.state()
        if self.carry.payload is None:raise RuntimeError('No current-session payload geometry; perceive or declare it again')
        self.carry.payload['gripper_rad']=state['right_gripper']
        self.carry.remove_source_obstacle()
        self.carry.status='attached';self.carry.reason='Operator confirmed attachment';self.carry._change()
        return self._status()

    async def set_geometry(self,args,invocation):
        from .payload import geometry
        state=self.state()
        if self.carry.status=='attached':self.carry.check(state)
        mount=np.eye(4);mount[:3,3]=[args[k] for k in ('x_m','y_m','z_m')]
        mount[:3,:3]=quaternion_matrix([args[k] for k in ('qx','qy','qz','qw')])
        payload={**(self.carry.payload or {'id':uuid.uuid4().hex}),'gripper_rad':state['right_gripper'],'T_tcp_object':mount.tolist(),
                 'size_m':[args[k] for k in ('size_x_m','size_y_m','size_z_m')],
                 'geometry_scope':'operator_declared_full_box'}
        self.carry.payload=geometry(payload);self.carry.remove_source_obstacle();self.carry.status='attached';self.carry.reason='Operator declared full geometry';self.carry._change()
        return self._status()

    async def plan_offset(self,args,invocation):
        args=deepcopy(args);tool=args['frame_id']=='a_right_tcp'
        args.update(relative=True,tool_relative=tool,path_mode='linear')
        if tool:args['frame_id']='a_right_base_link'
        return await self.plan(args,invocation)

    def _path(self,id):
        item=self.paths.get(id)
        if item is None or item['frozen'] or time.monotonic()-item['created']>300:
            raise RuntimeError('Path missing, frozen or expired; create a new path')
        return item

    async def create_path(self,args,invocation):
        while len(self.paths)>=8:self.paths.popitem(last=False)
        id=uuid.uuid4().hex
        self.paths[id]=dict(arguments=deepcopy(args),waypoints=[],created=time.monotonic(),frozen=False)
        return dict(path_id=id,waypoint_count=0)

    async def append_waypoint(self,args,invocation):
        item=self._path(args['path_id'])
        if len(item['waypoints'])>=32:raise ValueError('Path already has 32 waypoints')
        quaternion_matrix([args[k] for k in ('qx','qy','qz','qw')])
        item['waypoints'].append({k:v for k,v in args.items() if k!='path_id'})
        return dict(path_id=args['path_id'],waypoint_count=len(item['waypoints']))

    async def plan_path(self,args,invocation):
        item=self._path(args['path_id'])
        if not item['waypoints']:raise ValueError('Path is empty')
        item['frozen']=True
        return await self.plan({**item['arguments'],'waypoints':deepcopy(item['waypoints'])},invocation)

    async def plan_place(self,args,invocation):
        if self.carry.check(self.state())['status']!='attached':raise RuntimeError('Place requires a modeled carried object')
        return await self.plan({**args,'operation':'place','path_mode':'linear'},invocation)

    async def execute_place(self,args,invocation):
        entry=self.checked_plan(args['plan_id']);plan=entry['data']['plan']
        if plan.get('operation')!='place':raise ValueError('Not a placement plan')
        state=self.state();self.carry.check(state)
        if 'region_entry' in entry:self.check_region(entry['region_entry'],state)
        if self.carry.token!=entry['carry_token'] or not self.same_start(entry['state'],state):
            raise RuntimeError('Placement start or carried geometry changed; replan')
        await self.backend.grasp_check(self.profile)
        entry['consumed']=True
        payload_id=self.carry.payload['id']
        stage='transfer'
        directory=self.directory/args['plan_id']
        result=dict(placed=False,retracted=False,placement_verified=False,payload_id=payload_id,
                    release_started=False,release_confirmed=False,failed_stage='',
                    detail_ref=str(directory/'placement_state.json'))
        def record():
            (directory/'placement_state.json').write_text(json.dumps(
                dict(stage=stage,**result,carry=self.carry.snapshot()),allow_nan=False))
        async def move(segment,start):
            await self.backend.arm_trajectory(invocation,start['right'],segment['waypoints_rad'],segment['joint_speed_rad_s'])
        try:
            await self.guarded(lambda:move(plan['transfer'],state),
                lambda after:self.scene_matches(state,after) and abs(after['right_gripper']-state['right_gripper'])<=.02,
                message='Scene/gripper changed during transfer; no release')
            from .correction import correct_endpoint
            current,verified_transfer=await correct_endpoint(self,plan['transfer'],invocation,
                self.directory/args['plan_id']/'transfer_correction')
            stage='release'
            # A cancelled/failed opening leaves attachment uncertain, never empty.
            self.carry.uncertain('Release started; opening not yet confirmed')
            result['release_started']=True;record()
            from .release import ReleaseMonitor,confirm_opening
            monitor=ReleaseMonitor(self,current,verified_transfer,directory)
            def confirmed():
                result['release_confirmed']=True;record()
            released=await self.guarded(
                lambda:confirm_opening(self,invocation,plan['release_gap_m'],monitor,confirmed),
                monitor.check,message='Release hold check failed')
            footprint_tcp=np.linalg.inv(base_from_frame(self.urdf,released,'base_footprint'))@self.fk(released['right'])
            self.carry.released(footprint_tcp,plan.get('settling_drop_m',self.profile['release_clearance_m']+.005),released['wheels'])
            result['placed']=True
            stage='retreat'
            record()
            # Revalidate using actual release pose and the now static object.
            request=dict(directory='',urdf=self.urdf,state=released,profile=self.profile,
                         carry=self.carry.check(released),arguments=dict(side='right',frame_id=plan['place_frame'],
                         path_mode='linear',relative=True,tool_relative=False,dx_m=0.,dy_m=0.,dz_m=plan['retreat_height_m']))
            retreat_directory=directory/'actual_retreat';retreat_directory.mkdir()
            request['directory']=str(retreat_directory)
            (retreat_directory/'request.json').write_text(json.dumps(request,allow_nan=False))
            data=await self._worker(retreat_directory)
            if data['validation']['status']!='passed':raise RuntimeError('Actual retreat failed collision checks')
            if not self.same_start(released,self.state()):raise RuntimeError('Robot moved while revalidating retreat')
            await self.guarded(lambda:move(data['plan'],released),
                lambda after:self.scene_matches(released,after) and abs(after['right_gripper']-released['right_gripper'])<=.02,
                message='Scene/gripper changed during retreat')
            await correct_endpoint(self,data['plan'],invocation,
                self.directory/args['plan_id']/'retreat_correction')
            result['retracted']=True;stage='completed';record()
            return result
        except BaseException as exc:
            result['failed_stage']=stage
            try:
                record()
            except OSError:pass
            if isinstance(exc,Exception):
                from ..skills import SkillFailure
                progress=('Object release confirmed; retreat incomplete' if result['placed'] else
                          'Gripper opening confirmed; release pose unconfirmed; retreat not started' if result['release_confirmed'] else
                          'Release unconfirmed; retreat not started' if result['release_started'] else
                          'Transfer incomplete; release not started')
                raise SkillFailure(f'{progress}; failed_stage={stage}; carry_status={self.carry.status}; {exc}',result) from exc
            raise
