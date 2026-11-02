"""Kinematic transport, opening and retreat previews; never controls hardware."""
from copy import deepcopy
import time
import json
from pathlib import Path
import numpy as np
from .payload import placed_obstacle
from .geometry import base_from_frame,pose_fields
from .planner import make_plan,validate_arm_plan
from ..grasp.kinematics import RightArmFK


class PlacementPreflightError(ValueError):
    """Rejected terminal configurations; contains diagnostics, never an executable plan."""
    def __init__(self, result):
        super().__init__('Release configurations collide before trajectory generation')
        self.result=result


def release_preflight(request, planner, goal, candidates, carry, clearance):
    """Screen all distinct proposed release configurations before expensive path IK.

    Passing this screen never certifies a plan. Full transfer/opening/retreat
    validation remains mandatory after trajectory generation.
    """
    profile=request['profile'];args=request['arguments'];state=request['state']
    payload=carry['payload'];seen=[];first=None;plans=[]
    scene={**request['_collision_scene'],'release_object_body':'placed_'+payload['id']}
    opened=-args['release_gap_m']/abs(profile['gap_per_motor_rad_m'])
    if not -4.5<=opened<=0:raise ValueError('Release gap outside motor opening range')
    for candidate in candidates:
        try:q=planner.choose(goal,np.asarray(state['right']),[candidate])
        except ValueError:continue
        if any(np.max(np.abs(q-other))<1e-5 for other in seen):continue
        seen.append(q)
        actual=planner.fk(q)
        footprint_tcp=np.linalg.inv(base_from_frame(request['urdf'],state,'base_footprint'))@goal
        obstacle=placed_obstacle(payload,footprint_tcp,clearance+.005)
        empty={**carry,'status':'empty','payload':None,'obstacles':[*carry['obstacles'],obstacle]}
        plan=dict(schema='hei.arm_plan/v1',side='right',base_frame=planner.fk.base_frame,tool_frame=planner.fk.tool_frame,
            urdf_sha256=planner.fk.sha256,target_frame='a_right_base_link',target_pose=pose_fields(goal,'a_right_base_link'),
            T_start=actual.tolist(),T_goal=goal.tolist(),start_joint_rad=q.tolist(),start_gripper_rad=state['right_gripper'],
            waypoints_rad=[q.tolist()],joint_speed_rad_s=profile['joint_speed_rad_s'],path_mode='joint',
            nominal_duration_sec=0.,carry=empty,gripper_end_rad=opened,preflight_only=True)
        plans.append(plan)
    if not plans:return
    from ..collision import build_scene
    from ..collision.cuda_batch import posed_samples
    context=build_scene(request['urdf'],plans[0],scene)
    model,data=context
    # Keep the first per-candidate probe lazy: a quick pass can proceed directly
    # to its sweep. Cross-candidate batches are screened by batch_release_screen.
    # Subsequent validation reuses the exact contacts computed for that pose.
    rows=((float(i),'start',np.asarray(p['start_joint_rad']),opened) for i,p in enumerate(plans))
    for row in posed_samples(model,data,rows):
        plan=plans[int(row[0])]
        # Fully open is a mandatory endpoint. Reject it before sweeping all
        # intermediate finger positions; an endpoint pass still needs the sweep.
        endpoint={**plan,'start_gripper_rad':opened}
        endpoint.pop('gripper_end_rad')
        report=validate_arm_plan(endpoint,request['urdf'],scene,_collision_scene=context)
        report['preflight_phase']='fully_open'
        if report['status']!='failed':
            report=validate_arm_plan(plan,request['urdf'],scene,_collision_scene=context)
            report['preflight_phase']='opening_sweep'
        if report['status']!='failed':return
        if first is None:first=dict(plan=plan,validation=dict(status='failed',stages={'opening':report},
            unchecked_stages=['transfer','retreat'],preflight_only=True))
    if seen:
        first['validation']['release_configurations_checked']=len(seen)
        raise PlacementPreflightError(first)
    # No accurate proposal: let the established IK path/fallback produce its error.


def placement_rotation(request, start=None):
    """Explicitly permitted outward tilt around a horizontal shoulder tangent."""
    import cv2
    state,args=request['state'],request['arguments']
    frame=base_from_frame(request['urdf'],state,'base_footprint')
    if start is None:start=RightArmFK(request['urdf'])(state['right'])
    rotation=cv2.Rodrigues(frame[:3,2]*float(args.get('_placement_yaw_rad',0.)))[0]@start[:3,:3]
    tilt=float(args.get('_placement_tilt_rad',0.))
    limit=args.get('max_tilt_deg',10.)
    if type(limit) not in (float,int) or not np.isfinite(limit) or not 0<=limit<=10:
        raise ValueError('Placement max_tilt_deg must be within 0..10 degrees')
    if not np.isfinite(tilt) or abs(tilt)>np.deg2rad(limit)+1e-12:
        raise ValueError('Placement tilt exceeds explicitly permitted angle')
    if tilt:
        center=np.asarray(args['region']['T_footprint_region'])[:3,3]
        shoulder=np.linalg.inv(frame)[:3,3]
        radial=np.r_[center[:2]-shoulder[:2],0.]
        axis=np.cross(radial,[0.,0.,1.]);length=np.linalg.norm(axis)
        if length<1e-6:raise ValueError('Placement tilt direction is undefined')
        rotation=cv2.Rodrigues(frame[:3,:3]@(axis/length)*tilt)[0]@rotation
    return rotation


def placement_targets(request):
    state,profile,args=request['state'],request['profile'],request['arguments']
    carry=deepcopy(request['carry'])
    if carry['status']!='attached' or carry['payload'] is None:raise ValueError('Placement requires attached geometry')
    if len(carry['obstacles'])>=16:raise ValueError('Placed-object map full; reconcile scene before placing')
    payload=carry['payload'];mount=np.asarray(payload['T_tcp_object']);size=np.asarray(payload['size_m'])
    padding=profile['payload_padding_m']
    if args['release_gap_m']<=np.abs(mount[1,:3])@size+2*padding:
        raise ValueError('Release gap does not clear the padded object width')
    fk=RightArmFK(request['urdf']);start=fk(state['right'])
    frame=base_from_frame(request['urdf'],state,args['frame_id'])
    up=frame[:3,2]
    gravity_up=base_from_frame(request['urdf'],state,'base_footprint')[:3,2]
    if up@gravity_up<.999:raise ValueError('Placement frame must have gravity-aligned +Z')
    import cv2
    rotation=placement_rotation(request,start)
    object_rotation=rotation@mount[:3,:3]
    half_height=float(np.abs(up@object_rotation)@size/2)
    clearance=profile['release_clearance_m']
    if 'region' in args:
        clearance=max(clearance,2*padding+profile['scene'].get('clearance_m',.003)+.003)
    object_center=frame@np.array([args['x_m'],args['y_m'],args['support_z_m']+half_height+clearance,1.])
    if args.get('region',{}).get('relation')=='on':
        region=args['region']
        footprint=base_from_frame(request['urdf'],state,'base_footprint')
        local=np.linalg.inv(np.asarray(region['T_footprint_region']))@np.linalg.inv(footprint)@object_center
        # Uniform-density box center is the available COM estimate. User permits
        # overhang for on-placement; retain the existing center margin.
        allowance=np.asarray(region['size_xy_m'])/2-padding-.005
        if np.any(allowance<=0) or np.any(abs(local[:2])>allowance+1e-9):
            raise ValueError('Estimated object COM projection is outside the support interior')
    goal=start.copy();goal[:3,:3]=rotation;goal[:3,3]=object_center[:3]-rotation@mount[:3,3]
    high=max(float(up@start[:3,3]),float(up@goal[:3,3]))+args['approach_height_m']+args.get('region_wall_height_m',0.)
    raised=start.copy();raised[:3,3]+=up*(high-float(up@raised[:3,3]))
    above=goal.copy();above[:3,3]+=up*(high-float(up@above[:3,3]))
    return start,raised,above,goal,clearance


def make_place_plan(request):
    state,profile,args=request["state"],request["profile"],request["arguments"]
    carry=deepcopy(request["carry"]);payload=carry["payload"]
    start,raised,above,goal,clearance=placement_targets(request)
    up=base_from_frame(request["urdf"],state,"base_footprint")[:3,2]
    def make(start_state,targets,context):
        return make_plan({**request,'state':start_state,'carry':context,'arguments':dict(side='right',frame_id='a_right_base_link',
            path_mode='linear',_placement_soft_intermediate=True,waypoints=[pose_fields(t,'a_right_base_link') for t in targets])})
    # Only the release pose is an accuracy-constrained task endpoint.
    # Raised/above are generated transit points, not placement goals.
    from ..grasp.planner_backend import get_planner
    planner=get_planner(request['urdf'],profile.get('ik_backend','auto'))
    if getattr(planner,'backend','scipy')=='curobo':
        release_solutions=request.get('_release_solutions')
        if release_solutions is None:release_solutions=planner.proposals([goal],state['right'])[0]
        proposals=[None,None,release_solutions]
        for label,target,solutions in zip(('raise','above','release'),(raised,above,goal),proposals):
            if label != 'release':
                continue
            try:planner.choose(target,np.asarray(state['right']),solutions)
            except ValueError:
                try:
                    if hasattr(planner,'solve_proposed'):
                        refined=planner.solve_proposed(target,np.asarray(state['right']),solutions)
                    else:refined=planner.solve(target,np.asarray(state['right']))
                    # CPU refinement is an accurate release solution too. Keep
                    # it for opening preflight instead of discarding it and
                    # discovering the same failure after expensive path IK.
                    proposals[2]=np.concatenate([np.asarray(solutions),np.asarray(refined)[None]],axis=0)
                except ValueError as error:raise ValueError(f'{label} endpoint: {error}') from error
    if '_collision_scene' in request and getattr(planner,'backend','scipy')=='curobo':
        release_preflight(request,planner,goal,proposals[2],carry,clearance)
    transfer=make(state,[raised,above,goal],carry)
    end_state={**state,'right':transfer['waypoints_rad'][-1]}
    footprint_tcp=np.linalg.inv(base_from_frame(request['urdf'],state,'base_footprint'))@goal
    obstacle=placed_obstacle(payload,footprint_tcp,clearance+.005)
    obstacle['anchor_wheels']=list(state['wheels'])
    empty={**carry,'status':'empty','payload':None,'obstacles':[*carry['obstacles'],obstacle]}
    opening=make(end_state,[goal],empty)
    opened_rad=-args['release_gap_m']/abs(profile['gap_per_motor_rad_m'])
    if not -4.5<=opened_rad<=0:raise ValueError('Release gap outside motor opening range')
    opening['gripper_end_rad']=opened_rad
    opened_state={**end_state,'right_gripper':opened_rad}
    retreat=goal.copy();retreat[:3,3]+=up*args['retreat_height_m']
    retreat_plan=make(opened_state,[retreat],empty)
    result=deepcopy(transfer)
    result.update(placement_tilt_deg=float(np.degrees(args.get('_placement_tilt_rad',0.))),
                  operation='place',transfer=transfer,opening=opening,retreat=retreat_plan,
                  release_gap_m=args['release_gap_m'],retreat_height_m=args['retreat_height_m'],
                  place_frame=args['frame_id'],released_object_id=payload['id'],settling_drop_m=clearance+.005,
                  nominal_duration_sec=transfer['nominal_duration_sec']+5+retreat_plan['nominal_duration_sec'])
    if args.get('region',{}).get('relation')=='on':
        result.update(support_policy='estimated_box_com_inside_support',
            support_com_margin_m=profile['payload_padding_m']+.005,overhang_allowed=True)
    return result


def validate_place_plan(plan,urdf,scene,*,stop_on_failure=False):
    # Region search should reject an impossible opening before validating the
    # entire transfer and retreat. A passed result still checks every stage.
    stages={}
    order=('opening','transfer','retreat') if stop_on_failure else ('transfer','opening','retreat')
    for name in order:
        stage_scene=({**scene,'release_object_body':'placed_'+plan['released_object_id']}
                     if name=='opening' else scene)
        stages[name]=validate_arm_plan(plan[name],urdf,stage_scene)
        if stop_on_failure and stages[name]['status']=='failed':
            break
    status='failed' if plan['nominal_duration_sec']>500 or any(s['status']=='failed' for s in stages.values()) else (
        'passed' if len(stages)==3 and all(s['status']=='passed' for s in stages.values()) else 'incomplete')
    return dict(status=status,stages=stages,duration_limit_exceeded=plan['nominal_duration_sec']>500,
                unchecked_stages=[name for name in ('transfer','opening','retreat') if name not in stages])


def select_payload_region(region,urdf,state,payload,padding):
    """Select a supported rectangle whose aspect ratio fits the current payload."""
    if 'spatial_model' in region:return deepcopy(region)
    footprint=base_from_frame(urdf,state,'base_footprint')
    rotation=footprint[:3,:3].T@RightArmFK(urdf)(state['right'])[:3,:3]@np.asarray(payload['T_tcp_object'])[:3,:3]
    options=[{},*region.get('usable_rectangle_options',[])]
    for index,option in enumerate(options):
        candidate={**region,**option}
        extent=np.abs(np.asarray(candidate['T_footprint_region'])[:3,:3].T@rotation)@np.asarray(payload['size_m'])/2
        required_extent=np.zeros(2) if candidate['relation']=='on' else extent[:2]
        if np.all(np.asarray(candidate['size_xy_m'])/2-required_extent-padding-.005>0):
            candidate['selected_rectangle_option']=index
            return candidate
    return deepcopy(region)  # Existing fit check returns detailed dimensions.


def _region_place_candidates(request, yaw=0.):
    """Choose object-center positions within a rectangular usable region, no fabricated dimensions."""
    region=request['arguments']['region'];state=request['state']
    payload=request['carry']['payload']
    if request['carry']['status']!='attached' or payload is None:raise ValueError('Placement requires attached geometry')
    footprint=base_from_frame(request['urdf'],state,'base_footprint')
    pose=np.asarray(region['T_footprint_region'],float)
    import cv2
    yaw_rotation=cv2.Rodrigues(np.array([0.,0.,float(yaw)]))[0]
    oriented={**request,'arguments':{**request['arguments'],'_placement_yaw_rad':float(yaw)}}
    rotation=footprint[:3,:3].T@placement_rotation(oriented)@np.asarray(payload['T_tcp_object'])[:3,:3]
    extent=np.abs(pose[:3,:3].T@rotation)@np.asarray(payload['size_m'])/2
    margin=request['profile']['payload_padding_m']+.005
    required_extent=np.zeros(2) if region['relation']=='on' else extent[:2]
    available=np.asarray(region['size_xy_m'])/2-required_extent-margin
    if np.any(available<=0):
        required=2*(required_extent+margin)
        raise ValueError('Carried object does not fit within usable placement region: '
                         f'estimated usable size_m={region["size_xy_m"]}, '
                         f'required size_m={required.tolist()}, '
                         f'geometry_scope={region.get("geometry_scope","unknown")}; '
                         'this does not establish the physical container size')
    if 'spatial_model' in region:
        from .spatial import supported_centers
        from itertools import islice
        import cv2
        from itertools import product
        corners=np.array(list(product([-1.,1.],repeat=3)))*np.asarray(payload['size_m'])/2
        projected=(corners@(pose[:3,:3].T@rotation).T)[:,:2]
        # Minkowski sum with the same XY margin; avoid the fictitious corners
        # introduced by enclosing a rotated object in an axis-aligned rectangle.
        expanded=(projected[:,None,:]+np.array(list(product([-margin,margin],repeat=2)))[None,:,:]).reshape(-1,2)
        polygon=cv2.convexHull(expanded.astype(np.float32)).reshape(-1,2).astype(float)
        offsets=list(islice(supported_centers(region,extent[:2]+margin,available,polygon,diverse=True),8))
        if not offsets:
            data=region['spatial_model']['coverage']
            raise ValueError('Object fits container inner envelope but no support footprint satisfies coverage/occupancy policy: '
                             f'inner_size_m={region["size_xy_m"]}, '
                             f'max_unknown_support_fraction={region.get("max_unknown_support_fraction",0.)}, '
                             f'unknown_area_m2={data["unknown_area_m2"]}, '
                             f'occupied_area_m2={data["occupied_area_m2"]}; recapture or inspect coverage')
    else:
        offsets=[available*np.asarray(offset) for offset in ([0,0],[.5,0],[-.5,0],[0,.5],[0,-.5])]
    for offset in offsets:
        point=pose@np.array([*offset,0.,1.])
        args={**request['arguments'],'frame_id':'base_footprint','x_m':float(point[0]),'y_m':float(point[1]),
              'support_z_m':float(point[2]),'region_wall_height_m':region['wall_height_m'],
              '_placement_yaw_rad':float(yaw)}
        yield {**request,'arguments':args}



def region_place_candidates(request):
    """Try preserved inclination first, then tilt up to the default 10 degrees."""
    limit=request['arguments'].get('max_tilt_deg',10.)
    if type(limit) not in (float,int) or not np.isfinite(limit) or not 0<=limit<=10:
        raise ValueError('Placement max_tilt_deg must be within 0..10 degrees')
    yield from _yaw_region_place_candidates(request)
    for angle in dict.fromkeys((min(5.,limit),float(limit))):
        if angle<=0:continue
        tilted={**request,'arguments':{**request['arguments'],'_placement_tilt_rad':float(np.deg2rad(angle))}}
        yield from _yaw_region_place_candidates(tilted)


def _yaw_region_place_candidates(request):
    """Interleave positions and gravity-axis orientations within a bounded budget."""
    region=request['arguments']['region'];state=request['state']
    frame=base_from_frame(request['urdf'],state,'base_footprint')
    rotation=frame[:3,:3].T@placement_rotation(request)@np.asarray(request['carry']['payload']['T_tcp_object'])[:3,:3]
    region_rotation=np.asarray(region['T_footprint_region'])[:3,:3]
    # Include exact object-to-container alignment; fixed 45-degree increments
    # can miss a narrow but valid support interval at every sampled yaw.
    axis=int(np.argmax(np.asarray(request['carry']['payload']['size_m']) * np.linalg.norm(rotation[:2,:],axis=0)))
    aligned=np.arctan2(region_rotation[1,0],region_rotation[0,0])-np.arctan2(rotation[1,axis],rotation[0,axis])
    yaws=list(dict.fromkeys(round(float((yaw+np.pi)%(2*np.pi)-np.pi),10)
        for yaw in (aligned,aligned+np.pi/2,aligned+np.pi,aligned-np.pi/2,0.,np.pi/4,-np.pi/4,np.pi/2,-np.pi/2,np.pi)))
    groups=[iter(_region_place_candidates(request,yaw)) for yaw in yaws]
    errors=[];emitted=False;count=0
    while groups:
        active=[]
        for group in groups:
            try:
                candidate=next(group)
            except StopIteration:continue
            except ValueError as error:
                errors.append(error);continue
            active.append(group);emitted=True
            yield candidate
            count+=1
            if count>=32:return
        groups=active
    if not emitted:
        raise errors[0] if errors else ValueError('No supported placement pose')


def batch_release_screen(request,candidates,planner,scene,context_cache=None):
    """Screen all accurate release configurations in one shared scene batch.

    The newly released object is omitted ONLY from this rejection screen. A pass
    must still go through per-candidate opening/object and full trajectory checks.
    This avoids rebuilding a collision world/GPU buffers for each rejected pose.
    """
    from ..collision import build_scene
    from ..collision.cuda_batch import posed_samples
    state=request['state'];profile=request['profile']
    empty={**request['carry'],'status':'empty','payload':None}
    opened=-request['arguments']['release_gap_m']/abs(profile['gap_per_motor_rad_m'])
    if not -4.5<=opened<=0:raise ValueError('Release gap outside motor opening range')
    plans=[];owners=[];counts={};first={};passed=set()
    for index,candidate in enumerate(candidates):
        goal=placement_targets(candidate)[3];seen=[]
        for proposal in candidate['_release_solutions']:
            try:q=planner.choose(goal,np.asarray(state['right']),[proposal])
            except ValueError:continue
            if any(np.max(np.abs(q-other))<1e-5 for other in seen):continue
            seen.append(q)
            plans.append(dict(schema='hei.arm_plan/v1',side='right',base_frame=planner.fk.base_frame,
                tool_frame=planner.fk.tool_frame,urdf_sha256=planner.fk.sha256,
                T_start=planner.fk(q).tolist(),T_goal=goal.tolist(),start_joint_rad=q.tolist(),
                start_gripper_rad=opened,waypoints_rad=[q.tolist()],joint_speed_rad_s=profile['joint_speed_rad_s'],
                nominal_duration_sec=0.,carry=empty,preflight_only=True))
            owners.append(index)
        counts[index]=len(seen)
    if not plans:return dict(configurations=0,rejected_candidates=0)
    if context_cache is None:context_cache=[]
    if not context_cache:context_cache.append(build_scene(request['urdf'],plans[0],scene))
    context=context_cache[0]
    model,data=context
    queries_before=getattr(model,'query_count',0)
    rows=((float(i),'start',np.asarray(p['start_joint_rad']),opened) for i,p in enumerate(plans))
    for row in posed_samples(model,data,rows,first_cpu=False):
        i=int(row[0]);owner=owners[i]
        if owner in passed:continue
        report=validate_arm_plan(plans[i],request['urdf'],scene,_collision_scene=context)
        report['preflight_phase']='batched_fully_open'
        if report['status']!='failed':passed.add(owner)
        elif owner not in first:
            first[owner]=dict(plan=plans[i],validation=dict(status='failed',stages={'opening':report},
                unchecked_stages=['transfer','retreat'],preflight_only=True,
                preflight_scope='scene_without_released_object'))
    rejected=0
    for owner,result in first.items():
        if owner in passed:continue
        result['validation']['release_configurations_checked']=counts[owner]
        candidates[owner]['_release_rejection']=result;rejected+=1
    return dict(configurations=len(plans),rejected_candidates=rejected,
                collision_accelerator=getattr(model,'collision_accelerator','cpu_fcl'),
                narrow_phase_queries=getattr(model,'query_count',0)-queries_before)


def compute_region_place(request,scene):
    region=request['arguments']['region']
    if 'spatial_model' in region:
        from .regions import region_obstacles
        expected=region_obstacles(region,request['state']['wheels'])
        actual=[item for item in request['carry']['obstacles'] if item.get('region_id')==region['id']]
        # Sampling anchors can differ by encoder noise between state reads.
        # Scene motion is checked separately; compare only the geometry here.
        geometry=lambda items:[{k:v for k,v in item.items() if k!='anchor_wheels'} for item in items]
        def same(a,b):
            if isinstance(a,dict) and isinstance(b,dict):
                return a.keys()==b.keys() and all(same(a[k],b[k]) for k in a)
            if isinstance(a,list) and isinstance(b,list):
                return len(a)==len(b) and all(same(x,y) for x,y in zip(a,b))
            if isinstance(a,(int,float)) and isinstance(b,(int,float)):
                # Native and compute NumPy builds can differ at the last bit.
                return bool(np.isfinite(a) and np.isfinite(b) and abs(a-b)<=1e-12)
            return a==b
        if not same(geometry(actual),geometry(expected)):
            raise ValueError('Container collision model missing or stale; restart agent_bridge when idle and recapture region')
    attempts=[];first=None
    started=time.monotonic()
    candidates=list(region_place_candidates(request))
    if not candidates:raise ValueError('No reachable placement candidate: []')
    from ..grasp.planner_backend import get_planner
    planner=get_planner(request['urdf'],request['profile'].get('ik_backend','auto'))
    batch_report=None;context_cache=[]
    if candidates and getattr(planner,'backend','scipy')=='curobo':
        goals=[placement_targets(candidate)[3] for candidate in candidates]
        proposals=planner.proposals(goals,request['state']['right'])
        candidates=[{**candidate,'_release_solutions':solutions} for candidate,solutions in zip(candidates,proposals)]
        if hasattr(planner,'fk'):
            batch_report=dict(configurations=0,rejected_candidates=0,narrow_phase_queries=0,elapsed_sec=0.,batches=0)
    for candidate_index,candidate in enumerate(candidates):
        # Preserve the fast path when the first candidate already works.
        # Only build the shared rejection world after that direct attempt fails.
        if batch_report is not None and candidate_index>0 and (candidate_index-1)%4==0:
            batch_started=time.monotonic()
            report=batch_release_screen(request,candidates[candidate_index:candidate_index+4],planner,scene,context_cache)
            for field in ('configurations','rejected_candidates','narrow_phase_queries'):
                batch_report[field]+=report.get(field,0)
            batch_report['elapsed_sec']+=time.monotonic()-batch_started
            batch_report['batches']+=1
            if 'collision_accelerator' in report:
                batch_report['collision_accelerator']=report['collision_accelerator']
        candidate_started=time.monotonic()
        planning_elapsed=None
        try:
            if '_release_rejection' in candidate:
                raise PlacementPreflightError(candidate['_release_rejection'])
            plan=make_place_plan({**candidate,'_collision_scene':scene})
            planning_elapsed=time.monotonic()-candidate_started
            report=validate_place_plan(plan,request['urdf'],scene,stop_on_failure=True)
            result=dict(plan=plan,validation=report)
            # Report the candidate that progressed furthest, rather than hiding
            # a transfer failure behind the first rejected opening endpoint.
            if first is None or len(report.get('stages',{}))>len(first['validation'].get('stages',{})):
                first=result
            attempts.append(dict(x_m=candidate['arguments']['x_m'],y_m=candidate['arguments']['y_m'],status=report['status'],
                                 validation=report))
            if report['status']=='passed':
                result['placement_attempts']=attempts
                result['batch_release_screen']=batch_report
                plan['region_id']=request['arguments']['region']['id']
                return result
        except PlacementPreflightError as error:
            if first is None:first=error.result
            attempts.append(dict(status='failed',phase='release_preflight',error=str(error),validation=error.result['validation']))
        except ValueError as error:attempts.append(dict(status='failed',error=str(error)))
        finally:
            if attempts:
                attempts[-1].update(x_m=candidate['arguments']['x_m'],y_m=candidate['arguments']['y_m'],
                    yaw_rad=candidate['arguments'].get('_placement_yaw_rad',0.),
                    tilt_deg=float(np.degrees(candidate['arguments'].get('_placement_tilt_rad',0.))))
                attempts[-1]['elapsed_sec']=time.monotonic()-candidate_started
                if planning_elapsed is not None:
                    attempts[-1]['planning_sec']=planning_elapsed
                    attempts[-1]['validation_sec']=attempts[-1]['elapsed_sec']-planning_elapsed
            if request.get('directory'):
                path=Path(request['directory'])/'placement_progress.json'
                path.parent.mkdir(parents=True,exist_ok=True)
                tmp=path.with_suffix('.tmp')
                tmp.write_text(json.dumps(dict(elapsed_sec=time.monotonic()-started,attempts=attempts)))
                tmp.replace(path)
    if first:
        first['placement_attempts']=attempts
        first['batch_release_screen']=batch_report
        return first
    raise ValueError('No reachable placement candidate: '+str(attempts))
