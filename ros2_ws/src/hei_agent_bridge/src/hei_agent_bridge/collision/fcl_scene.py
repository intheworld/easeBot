"""Cached URDF convex geometry and FCL narrow phase with vectorized AABB culling.

No simulator, hardware, ROS, or CUDA dependency. Geometry is cached by file version;
scene transforms and contact policy state belong to each validation, never cached.
"""
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET
import cv2
import json
import numpy as np
from scipy.spatial import ConvexHull
from .convex import LazyPolytope, box_polytope



@lru_cache(maxsize=32)
def collision_pairs(bodies,moving,weld_items,parent_items,root):
    """Cache topology only; transforms, broad phase and contacts remain per pose."""
    weld=dict(weld_items);parent_group=dict(parent_items);moving=set(moving)
    dynamic_ids=[i for i,body in enumerate(bodies) if body in moving]
    dynamic_set=set(dynamic_ids);pairs=[]
    for i in dynamic_ids:
        wa=weld.get(bodies[i],bodies[i])
        for j,body in enumerate(bodies):
            if j==i or (j in dynamic_set and j>i):continue
            wb=weld.get(body,body)
            if wa==wb:continue
            if (parent_group.get(wa)==wb and wb!=root) or (parent_group.get(wb)==wa and wa!=root):continue
            pairs.append((j,i))
    result=np.array(pairs,dtype=int).reshape(-1,2)
    result.setflags(write=False)
    return result


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        r, p, y = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        t[:3, :3] = cv2.Rodrigues(np.array([0., 0., y]))[0] @ cv2.Rodrigues(np.array([0., p, 0.]))[0] @ cv2.Rodrigues(np.array([r, 0., 0.]))[0]
    if not np.isfinite(t).all():
        raise ValueError('Nonfinite URDF origin')
    return t


def mesh_path(urdf, filename):
    if filename.startswith('package://'):
        package, relative = filename[10:].split('/', 1)
        if package != 'hei_description':
            raise ValueError('Unsupported collision mesh package: '+package)
        path = urdf.parent.parent / relative
    else:
        path = urdf.parent / filename
    return path.resolve(strict=True)


def robot_model(urdf):
    path = Path(urdf).resolve(strict=True)
    root = ET.parse(path).getroot()
    files = [path] + [mesh_path(path, m.get('filename')) for m in root.findall('link/collision/geometry/mesh')]
    versions = tuple((str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in files)
    return _robot_model(versions)


@lru_cache(maxsize=4)
def _robot_model(versions):
    import fcl
    from .meshes import stl_components
    root = ET.parse(versions[0][0]).getroot()
    urdf = Path(versions[0][0])
    links = [link.get('name') for link in root.findall('link')]
    joints = []
    pending = list(root.findall('joint'))
    children = {j.find('child').get('link') for j in pending}
    roots = set(links)-children
    if len(roots) != 1:
        raise ValueError('Collision URDF needs one root')
    known = set(roots)
    weld = {next(iter(roots)): next(iter(roots))}
    parent_group = {}
    moving = set()
    while pending:
        available = [j for j in pending if j.find('parent').get('link') in known]
        if not available:
            raise ValueError('Disconnected collision URDF')
        for j in available:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            kind = j.get('type'); name = j.get('name')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('Unsupported collision joint type: '+kind)
            axis = np.fromstring(j.find('axis').get('xyz'), sep=' ') if kind != 'fixed' else np.zeros(3)
            if kind != 'fixed' and (axis.shape != (3,) or not np.isclose(np.linalg.norm(axis), 1)):
                raise ValueError('Invalid joint axis')
            limit = j.find('limit')
            bounds = None if kind in ('fixed', 'continuous') else (float(limit.get('lower')),float(limit.get('upper')))
            dynamic = name.startswith('a_right_joint') or name.startswith('a_right_end_joint_') or parent in moving
            if dynamic: moving.add(child)
            weld[child] = weld[parent] if kind == 'fixed' else child
            if kind != 'fixed': parent_group[child] = weld[parent]
            joints.append((name, parent, child, kind, origin(j.find('origin')), axis, bounds, dynamic))
            known.add(child); pending.remove(j)
    geometries = []
    for link in root.findall('link'):
        body = link.get('name')
        for index, collision in enumerate(link.findall('collision')):
            g = collision.find('geometry'); local = origin(collision.find('origin'))
            mesh = g.find('mesh')
            if mesh is not None:
                scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
                if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale<=0):
                    raise ValueError('Invalid collision mesh scale')
                pieces = stl_components(mesh_path(urdf,mesh.get('filename')))
                for part, points in enumerate(pieces):
                    points = points*scale
                    hull = ConvexHull(points)
                    triangles = hull.simplices.copy()
                    # FCL requires outward-oriented convex polygons.
                    normals = np.cross(points[triangles[:,1]]-points[triangles[:,0]], points[triangles[:,2]]-points[triangles[:,0]])
                    flip = np.sum(normals*hull.equations[:,:3],axis=1)<0
                    triangles[flip] = triangles[flip][:,[0,2,1]]
                    used = np.unique(triangles)
                    remap = np.zeros(len(points),dtype=np.int64); remap[used]=np.arange(len(used))
                    faces = np.column_stack([np.full(len(triangles),3),remap[triangles]]).ravel()
                    shape = fcl.Convex(points[used],len(triangles),faces)
                    geometries.append((body,f'{body}_collision{index}_part{part}',local,shape,points.min(0),points.max(0),LazyPolytope(points,hull) if len(hull.vertices)<=128 else None))
            elif g.find('box') is not None:
                size=np.fromstring(g.find('box').get('size'),sep=' ')
                geometries.append((body,f'{body}_collision{index}',local,fcl.Box(*size),-size/2,size/2,box_polytope(size)))
            else:
                raise ValueError('Unsupported collision geometry; provide convex STL or box')
    return links,joints,geometries,weld,parent_group,moving,next(iter(roots))


def context_signature(urdf, plan, scene):
    path=Path(urdf).resolve();stat=path.stat()
    return (str(path),stat.st_mtime_ns,stat.st_size,
            json.dumps([plan.get('carry'),scene],sort_keys=True,allow_nan=False))


class FCLScene:
    backend = 'fcl'

    def __init__(self, urdf, plan, scene):
        import fcl
        self.fcl = fcl
        self.signature = context_signature(urdf,plan,scene)
        links,self.joints,geometries,weld,parent_group,moving,root = robot_model(urdf)
        self.names = list(links); self.transforms={root:np.eye(4)}
        left_q=np.asarray(scene.get('left_joint_rad') or [0.,-.5,-.5,0.,0.,0.],float)
        if left_q.shape!=(6,) or not np.isfinite(left_q).all():raise ValueError('Expected six finite left joint angles')
        values={f'b_left_joint{i+1}':v for i,v in enumerate(left_q)}
        values['lift_joint']=float(scene.get('lift_joint_m') or 0.)
        left=float(scene.get('left_finger_opening_m') or 0.)
        values.update(b_left_end_joint_L_finger=left,b_left_end_joint_R_finger=-left)
        self.values=values
        for name,parent,child,kind,local,axis,bounds,dynamic in self.joints:
            value=float(values.get(name,0.))
            # These are observed scene joints, not commanded trajectory goals.
            # Preserve their measured geometry; their controllers own travel
            # limits. In particular, lift zero noise must not reject arm plans.
            if not np.isfinite(value):
                raise ValueError('Non-finite scene joint feedback: '+name)
            self.transforms[child]=self.transforms[parent]@local@self.motion(kind,axis,value)
        self.geometries=list(geometries)
        self.moving=set(moving)
        self.extra={}
        clearance=float(scene.get('clearance_m',.003))
        if not np.isfinite(clearance) or not 0<=clearance<=.03:raise ValueError('Invalid collision clearance')
        self.clearance=clearance
        padding=float(scene.get('payload_padding_m',.003))
        if not np.isfinite(padding) or not 0<=padding<=.03:raise ValueError('Invalid payload padding')
        def box(parent,name,t,size,pad=0.):
            from ..grasp.transforms import validate_transforms
            t=validate_transforms(np.asarray(t,float)[None],name)[0]
            size=np.asarray(size,float)
            if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):raise ValueError('Invalid object dimensions')
            if parent not in self.transforms or name in self.names:raise ValueError('Invalid collision body '+name)
            self.names.append(name);self.extra[name]=(parent,t);self.transforms[name]=self.transforms[parent]@t
            if parent in self.moving:self.moving.add(name)
            size=size+2*pad
            self.geometries.append((name,name if name=='validation_table' else name+'_box',np.eye(4),fcl.Box(*size),-size/2,size/2,box_polytope(size)))
        if scene.get('table_top_z_m') is not None:
            t=np.eye(4);thick=float(scene.get('table_thickness_m',.04))
            xy=np.asarray(scene.get('table_center_xy_m',[.92,0]),float)
            size=np.asarray(scene.get('table_size_xy_m',[.96,1.24]),float)
            if xy.shape!=(2,) or size.shape!=(2,) or thick<=0:raise ValueError('Invalid table dimensions')
            t[:3,3]=[*xy,float(scene['table_top_z_m'])-thick/2]
            frame=scene.get('table_frame','a_right_base_link')
            if frame not in ('a_right_base_link','base_footprint'):raise ValueError('Invalid table frame')
            box(frame,'validation_table',t,[*size,thick])
        carry=plan.get('carry') or {};payload=carry.get('payload')
        if payload is not None:
            if carry.get('status')!='attached':raise ValueError('Cannot validate unknown payload')
            box('a_right_tcp','a_right_payload',payload['T_tcp_object'],payload['size_m'],padding)
        obstacles=carry.get('obstacles',[])
        if len(obstacles)>16:raise ValueError('Too many placed objects; reconcile scene')
        for item in obstacles:
            if item.get('kind')=='place_region_coverage':
                if len(item['boxes'])>4096:raise ValueError('Container coverage too fragmented')
                for i,b in enumerate(item['boxes']):
                    if b['kind'] not in ('unknown','occupied'):raise ValueError('Invalid coverage exclusion kind')
                    box('base_footprint','placed_'+item['id']+'_'+b['kind']+'_'+str(i),np.asarray(item['T_footprint_object'])@np.asarray(b['T_region_box']),b['size_m'],padding)
            else:box('base_footprint','placed_'+item['id'],item['T_footprint_object'],item['size_m'],padding)
        self.names.append('world');self.transforms['world']=np.eye(4)
        self.geometries.append(('world','validation_ground',np.eye(4),fcl.Halfspace(np.array([0.,0.,1.]),0.),np.full(3,-1e10),np.array([1e10,1e10,0.]),None))
        self.ids={name:i for i,name in enumerate(self.names)}
        self.geom_bodyid=np.array([self.ids[g[0]] for g in self.geometries])
        self.objects=[fcl.CollisionObject(g[3]) for g in self.geometries]
        self.pairs=collision_pairs(tuple(g[0] for g in self.geometries),
            tuple(sorted(self.moving)),tuple(sorted(weld.items())),tuple(sorted(parent_group.items())),root)
        self.data=SimpleNamespace(contact=[],ncon=0,xpos=np.zeros((len(self.names),3)))
        self.query_count=0
        self.local=np.asarray([g[2] for g in self.geometries])
        self.centers=np.asarray([(g[4]+g[5])/2 for g in self.geometries])
        self.extents=np.asarray([(g[5]-g[4])/2 for g in self.geometries])
        self.update_objects()

    @staticmethod
    def motion(kind,axis,value):
        t=np.eye(4)
        if kind in ('revolute','continuous'):t[:3,:3]=cv2.Rodrigues(axis*value)[0]
        elif kind=='prismatic':t[:3,3]=axis*value
        return t

    def body(self,key):
        i=self.ids[key] if isinstance(key,str) else int(key)
        return SimpleNamespace(id=i,name=self.names[i])

    def geom(self,index):return SimpleNamespace(name=self.geometries[index][1])

    def update_objects(self):
        self._contact_pose=None
        poses=np.asarray([self.transforms[name] for name in self.names])
        self.geom_poses=poses[self.geom_bodyid]@self.local
        rotations=self.geom_poses[:,:3,:3]
        centers=np.einsum('nij,nj->ni',rotations,self.centers)+self.geom_poses[:,:3,3]
        extents=np.einsum('nij,nj->ni',np.abs(rotations),self.extents)
        self.bounds=np.stack([centers-extents,centers+extents],axis=1)
        self.data.xpos[:]=poses[:,:3,3]

    def update_transforms(self,q,grip):
        self._contact_pose=None
        if np.asarray(q).shape!=(6,) or not np.isfinite(q).all():raise ValueError('Expected six finite joint angles')
        self.values.update({f'a_right_joint{i+1}':v for i,v in enumerate(q)})
        opened=float(np.clip(-grip/4.5,0,1))*.05
        self.values.update(a_right_end_joint_L_finger=opened,a_right_end_joint_R_finger=-opened)
        for name,parent,child,kind,local,axis,bounds,dynamic in self.joints:
            if dynamic:self.transforms[child]=self.transforms[parent]@local@self.motion(kind,axis,self.values.get(name,0.))
        for name,(parent,t) in self.extra.items():self.transforms[name]=self.transforms[parent]@t

    def set_pose(self,q,grip):
        key=(tuple(np.asarray(q,float)),float(grip))
        if getattr(self,"_contact_pose",None)==key:return
        self.update_transforms(q,grip)
        self.update_objects()
        self.contacts()
        self._contact_pose=key

    def tcp(self):return np.linalg.inv(self.transforms['a_right_base_link'])@self.transforms['a_right_tcp']

    def contacts(self,candidates=None):
        self._contact_pose=None
        fcl=self.fcl; contacts=[]
        if candidates is None:
            a,b=self.pairs.T
            gap=np.maximum(self.bounds[a,0]-self.bounds[b,1],self.bounds[b,0]-self.bounds[a,1])
            near=np.all(gap<=self.clearance+1e-9,axis=1)
            candidates=self.pairs[near]
        # Only narrow-phase candidates need native collision-object transforms.
        for index in np.unique(candidates):
            t=self.geom_poses[index]
            self.objects[index].setTransform(fcl.Transform(t[:3,:3].copy(),t[:3,3].copy()))
        for i,j in candidates:
            self.query_count+=1
            result=fcl.CollisionResult()
            fcl.collide(self.objects[i],self.objects[j],fcl.CollisionRequest(num_max_contacts=32,enable_contact=True),result)
            if result.is_collision:
                if not result.contacts:raise RuntimeError('FCL returned collision without contact depth')
                distance=-max(float(c.penetration_depth) for c in result.contacts)
            elif self.clearance>0:
                distance=fcl.distance(self.objects[i],self.objects[j],fcl.DistanceRequest(),fcl.DistanceResult())
            else:continue
            if not np.isfinite(distance):raise RuntimeError('Nonfinite collision distance')
            if distance<=self.clearance:
                contacts.append(SimpleNamespace(geom1=int(i),geom2=int(j),dist=float(distance)))
        self.data.contact=contacts;self.data.ncon=len(contacts)
