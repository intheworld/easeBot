"""Bounded robot-owned placement regions and composable scalar MHS skills."""
from collections import OrderedDict
from copy import deepcopy
import json
import time
import uuid

import numpy as np
from .geometry import FRAMES, base_from_frame


def region_geometry(frame, x, y, z, width, length, yaw, relation, wall_height, thickness):
    values=np.asarray([x,y,z,width,length,yaw,wall_height,thickness],float)
    if not np.isfinite(values).all() or min(width,length)<=.01 or max(width,length)>2:
        raise ValueError('Invalid region dimensions or coordinates')
    if relation not in ('on','inside') or not .001<=thickness<=.1:
        raise ValueError('Invalid region relation or boundary thickness')
    if relation=='inside' and not .01<=wall_height<=.5:
        raise ValueError('Container needs a measured positive wall height')
    if relation=='on' and wall_height!=0:raise ValueError('Surface region wall height must be zero')
    if frame[:3,2]@np.array([0.,0.,1.])<.999:
        raise ValueError('Region support must be horizontal in base_footprint')
    c,s=np.cos(yaw),np.sin(yaw)
    local=np.eye(4);local[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];local[:3,3]=[x,y,z]
    return dict(schema='hei.place_region/v1',frame_id='base_footprint',relation=relation,
                T_footprint_region=(frame@local).tolist(),size_xy_m=[width,length],
                wall_height_m=wall_height,boundary_thickness_m=thickness,
                geometry_scope='declared_rectangular_region')


def region_obstacles(region,wheels):
    pose=np.asarray(region['T_footprint_region']);w,l=region['size_xy_m'];t=region['boundary_thickness_m'];h=region['wall_height_m']
    xmin,ymin,xmax,ymax=region.get('outer_bounds_xy_m',[-w/2-t,-l/2-t,w/2+t,l/2+t])
    boxes=[('floor',[xmax-xmin,ymax-ymin,t],[(xmin+xmax)/2,(ymin+ymax)/2,-t/2])]
    if region['relation']=='inside':
        boxes += [('left',[-w/2-xmin,ymax-ymin,h],[(xmin-w/2)/2,(ymin+ymax)/2,h/2]),
                  ('right',[xmax-w/2,ymax-ymin,h],[(xmax+w/2)/2,(ymin+ymax)/2,h/2]),
                  ('front',[w,-l/2-ymin,h],[0,(ymin-l/2)/2,h/2]),
                  ('back',[w,ymax-l/2,h],[0,(ymax+l/2)/2,h/2])]
    result=[]
    for name,size,center in boxes:
        transform=pose.copy();transform[:3,3]=pose[:3,:3]@center+pose[:3,3]
        result.append(dict(id=region['id']+'_'+name,kind='place_region',region_id=region['id'],
                           size_m=size,T_footprint_object=transform.tolist(),anchor_wheels=list(wheels)))
    if 'spatial_model' in region:
        from .spatial import exclusion_boxes
        exclusions=exclusion_boxes(region)
        if exclusions:
            result.append(dict(id=region['id']+'_coverage',kind='place_region_coverage',
                               region_id=region['id'],T_footprint_object=pose.tolist(),
                               boxes=exclusions,anchor_wheels=list(wheels)))
    return result


class PlacementRegions:
    def register_regions(self,skills,outputs):
        self.regions=OrderedDict()
        number=lambda **kw:dict(type='number',**kw)
        result={'region_id':{'type':'string'},'region_ref':{'type':'string'},'relation':{'type':'string'},
                'frame_id':{'type':'string'},'geometry_scope':{'type':'string'},'expires_in_sec':number()}
        skills.register('create_place_region',
            'Operator-declared measured rectangular support region, no motion. x/y/support_z in frame_id, '
            'size_x/y are usable INNER dimensions. yaw rotates region axes around +Z. For inside provide measured '
            'wall_height and boundary_thickness; on requires wall_height=0. Do not invent dimensions or confirmation.',
            {'frame_id':{'type':'string','enum':list(FRAMES)},'relation':{'type':'string','enum':['on','inside']},
             'geometry_confirmed':{'type':'boolean','enum':[True]},
             **{k:number() for k in ('x_m','y_m','support_z_m','yaw_rad')},
             **{k:number(minimum=.011,maximum=2) for k in ('size_x_m','size_y_m')},
             'wall_height_m':number(minimum=0,maximum=.5),'boundary_thickness_m':number(minimum=.001,maximum=.1)},
            result,self.create_region)
        if self.region_perception is not None:
            skills.register('perceive_place_region',
                'From fresh front RGB-D and a target description, perceive a horizontal support (on) or an open '
                'rectangular container (inside). Requires visible support/bottom and, for inside, rim evidence on all '
                'four sides. No hidden-bottom inference. Returns robot-owned region_id; expires 180s after capture. No motion.',
                {'observation_id':{'type':'string'},'target':{'type':'string'},
                 'relation':{'type':'string','enum':['on','inside']}},result,self.perceive_region,timeout=90)
        skills.register('plan_object_placement',
            'Choose a position for the currently carried object in a recent region_id, then validate lift, transfer, '
            'descent, opening and retreat including region floor/walls. Try preserved inclination first, then allow up to 10 degrees of additional outward tilt. '
            'Defaults when unspecified by the user: approach_height_m=0.05, retreat_height_m=0.05, release_gap_m=0.05 (meters). '
            'Include these defaults explicitly in generated graph arguments; user-specified values override them. '
            'For on, overhang is allowed when estimated uniform-box COM stays inside the support with a margin; '
            'inside requires the full padded footprint to fit. '
            'No motion; execute_place_plan only if validated=true. No automatic obstacle detours or force placement.',
            {'side':{'type':'string','enum':['right']},'region_id':{'type':'string'},
             'approach_height_m':number(minimum=.01,maximum=.3),
             'retreat_height_m':number(minimum=.01,maximum=.3),'release_gap_m':number(minimum=.001,maximum=.1)},
            outputs,self.plan_region,timeout=180)

        skills.register('plan_object_placement_with_tilt',
            'Plan placement for an object that may be tilted: first try preserved inclination and yaw, then '
            'outward tilt up to max_tilt_deg (0..10 degrees). Caller must explicitly permit tilt; '
            'unsuitable for contents that must stay upright. Recompute tilted footprint, support height, '
            'carried-object geometry, opening and retreat collisions. '
            'Defaults when unspecified by the user: approach_height_m=0.05, retreat_height_m=0.05, release_gap_m=0.05 (meters), max_tilt_deg=10. '
            'Include these defaults explicitly in generated graph arguments; user-specified values override them. '
            'For on, overhang is allowed with estimated uniform-box COM inside support; inside requires full padded footprint. '
            'No motion; execute only if validated=true.',
            {'side':{'type':'string','enum':['right']},'region_id':{'type':'string'},
             'approach_height_m':number(minimum=.01,maximum=.3),'retreat_height_m':number(minimum=.01,maximum=.3),
             'release_gap_m':number(minimum=.001,maximum=.1),'max_tilt_deg':number(minimum=0,maximum=10)},
            outputs,self.plan_region,timeout=180)

    def _remember_region(self,region,state,created,ttl,directory=None):
        region=deepcopy(region);region['id']=uuid.uuid4().hex
        while len(self.regions)>=8:self.regions.popitem(last=False)
        if directory is None:
            directory=self.directory/('region_'+region['id']);directory.mkdir(parents=True)
        path=directory/'region.json';path.write_text(json.dumps(region,allow_nan=False))
        self.regions[region['id']]=dict(region=region,state=deepcopy(state),created=created,ttl=ttl)
        return dict(region_id=region['id'],region_ref=str(path),relation=region['relation'],frame_id=region['frame_id'],
                    geometry_scope=region['geometry_scope'],expires_in_sec=max(0.,ttl-(time.monotonic()-created)))

    async def create_region(self,args,invocation):
        if args['geometry_confirmed'] is not True:raise ValueError('Measured geometry confirmation required')
        state=self.state()
        frame=np.linalg.inv(base_from_frame(self.urdf,state,'base_footprint'))@base_from_frame(self.urdf,state,args['frame_id'])
        region=region_geometry(frame,*[args[k] for k in ('x_m','y_m','support_z_m','size_x_m','size_y_m','yaw_rad',
                                                        'relation','wall_height_m','boundary_thickness_m')])
        return self._remember_region(region,state,time.monotonic(),300)

    async def perceive_region(self,args,invocation):
        service=self.region_perception
        if service.computing:raise RuntimeError('A perception computation is already running')
        service.computing=True
        try:
            _,directory,request,state,created=service.capture_request(args)
            request.update(operation='perceive_region',relation=args['relation'])
            (directory/'request.json').write_text(json.dumps(request,allow_nan=False))
            result=await service._worker(directory)
            if not self.scene_matches(state,self.state()):raise RuntimeError('Scene moved during perception; recapture')
            if service._binding()!=service.binding:raise RuntimeError('Perception calibration changed; restart')
            if time.monotonic()-created>60:raise RuntimeError('Region observation expired; recapture')
            return self._remember_region(result['region'],state,created,180,directory)
        finally:service.computing=False

    def check_region(self,entry,state):
        if time.monotonic()-entry['created']>entry['ttl']:raise RuntimeError('Placement region expired; recapture or redeclare')
        if not self.scene_matches(entry['state'],state):raise RuntimeError('Region scene moved; recapture or redeclare')
        if self.region_perception and self.region_perception._binding()!=self.region_perception.binding:
            raise RuntimeError('Placement perception calibration changed; restart')

    async def plan_region(self,args,invocation):
        if self.computing:raise RuntimeError('An arm plan is already being computed')
        entry=self.regions.get(args['region_id'])
        if entry is None:raise RuntimeError('Unknown placement region; perceive or create it first')
        state=self.state();self.check_region(entry,state)
        if self.carry.check(state)['status']!='attached':raise RuntimeError('Placement requires a modeled carried object')
        from .place import select_payload_region
        selected=select_payload_region(entry['region'],self.urdf,state,self.carry.payload,
                                       self.profile['payload_padding_m'])
        if 'spatial_model' in selected:
            selected['max_unknown_support_fraction']=self.profile.get('max_unknown_support_fraction',0.)
        # Use the same selected geometry for planning, collision walls and memory.
        obstacles=region_obstacles(selected,state['wheels'])
        existing=[o for o in self.carry.obstacles if o.get('region_id')!=args['region_id']]
        if len(existing)+len(obstacles)>=16:raise RuntimeError('Scene map full; reconcile scene before placing')
        self.carry.obstacles=existing+obstacles;self.carry._change()
        result=await self.plan({**args,'operation':'place','path_mode':'linear','frame_id':'base_footprint',
                               'region':selected},invocation)
        planned=self.plans[result['plan_id']]
        planned['region_entry']=deepcopy(entry)
        if not result['validated']:
            report=planned['data']['validation']
            reasons=[]
            for stage,details in report.get('stages',{}).items():
                if details['status']!='failed':continue
                pairs=details.get('collision_pairs',[])
                if pairs:
                    pair=pairs[0]
                    reasons.append(f"{stage}: {pair.get('constraint_kind','collision')} between "
                                   + ' / '.join(pair['bodies']))
                reasons.extend(details.get('errors',[]))
            raise RuntimeError('No validated placement candidate; '+('; '.join(reasons) or report['status'])
                               +f"; details: {result['plan_ref']}")
        self.check_region(entry,self.state())
        return result
