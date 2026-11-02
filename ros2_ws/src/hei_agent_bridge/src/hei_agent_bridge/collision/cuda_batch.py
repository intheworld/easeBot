"""Batched CUDA FK/AABB broad phase; FCL remains the exact narrow phase.

Float64 and an outward numerical margin keep broad-phase culling conservative.
All geometry and topology come from the same FCL scene, including moving fingers
and carried objects. This does not substitute sphere approximations or certify
unsampled motion. No device calls are made by this module.
"""
from itertools import islice
import os
import numpy as np


class CudaBatch:
    def __init__(self, scene, *, device='cuda'):
        import torch
        self.torch = torch
        self.scene = scene
        self.device = device
        def tensor(value, dtype=torch.float64):
            return torch.as_tensor(np.asarray(value).copy(), dtype=dtype, device=device)
        self.tensor = tensor
        self.initial = tensor([scene.transforms[n] for n in scene.names])
        self.body_ids = tensor(scene.geom_bodyid, torch.long)
        self.local = tensor(scene.local)
        self.centers = tensor(scene.centers)
        self.extents = tensor(scene.extents)
        self.pairs = tensor(scene.pairs, torch.long)
        # Batch box/polytope pairs by the shared polytope. This is a complete
        # convex SAT screen, retaining the original mesh hull vertices/axes.
        self.sat_groups=[]
        groups={}
        for pair_index,(a,b) in enumerate(scene.pairs):
            ga=scene.geometries[a][6] if len(scene.geometries[a])>6 else None
            gb=scene.geometries[b][6] if len(scene.geometries[b])>6 else None
            if ga is None or gb is None:continue
            if ga['kind']=='box':a,b=b,a;ga,gb=gb,ga
            if gb['kind']!='box':continue
            groups.setdefault(int(a),[]).append((pair_index,int(b)))
        for a,rows in groups.items():
            shape=scene.geometries[a][6]
            if len(shape['vertices'])*(len(shape['normals'])+3*len(shape['edges'])+3)>65536:
                continue  # Large rounded CAD hulls remain on FCL; bound GPU memory.
            self.sat_groups.append((a,tensor([r[0] for r in rows],torch.long),
                tensor([r[1] for r in rows],torch.long),tensor(shape['vertices']),
                tensor(shape['normals']),tensor(shape['edges'])))
        self.joints = []
        for name, parent, child, kind, local, axis, _, dynamic in scene.joints:
            if not dynamic:
                continue
            index = next((i for i in range(6) if name == f'a_right_joint{i+1}'), None)
            sign = 1 if name == 'a_right_end_joint_L_finger' else -1 if name == 'a_right_end_joint_R_finger' else 0
            x,y,z = axis
            skew = tensor([[0,-z,y],[z,0,-x],[-y,x,0]])
            self.joints.append((scene.ids[parent],scene.ids[child],kind,tensor(local),
                                tensor(axis),skew,skew@skew,index,sign,scene.values.get(name,0.)))
        self.extra = [(scene.ids[n],scene.ids[p],tensor(t)) for n,(p,t) in scene.extra.items()]

    def compute(self, qs, grips):
        torch = self.torch
        qs = np.asarray(qs, float); grips = np.asarray(grips, float)
        if qs.ndim != 2 or qs.shape[1] != 6 or grips.shape != (len(qs),) or not np.isfinite(qs).all() or not np.isfinite(grips).all():
            raise ValueError('Expected finite batched joint and gripper positions')
        with torch.inference_mode():
            q = self.tensor(qs); opened = self.tensor(np.clip(-grips/4.5,0,1)*.05)
            count = len(qs)
            poses = self.initial.expand(count,-1,-1,-1).clone()
            identity = torch.eye(4,dtype=q.dtype,device=self.device).expand(count,4,4)
            for parent,child,kind,local,axis,skew,skew2,index,sign,value in self.joints:
                motion = identity.clone()
                angle = q[:,index] if index is not None else opened*sign if sign else torch.full((count,),value,dtype=q.dtype,device=self.device)
                if kind in ('revolute','continuous'):
                    motion[:,:3,:3] += torch.sin(angle)[:,None,None]*skew + (1-torch.cos(angle))[:,None,None]*skew2
                elif kind == 'prismatic':
                    motion[:,:3,3] = angle[:,None]*axis
                poses[:,child] = poses[:,parent] @ local @ motion
            for child,parent,local in self.extra:
                poses[:,child] = poses[:,parent] @ local
            geoms = poses[:,self.body_ids] @ self.local
            rot = geoms[:,:,:3,:3]
            centers = (rot @ self.centers[...,None]).squeeze(-1)+geoms[:,:,:3,3]
            extents = (rot.abs() @ self.extents[...,None]).squeeze(-1)
            low,high = centers-extents,centers+extents
            # Bound temporary memory even with thousands of coverage boxes.
            masks = []
            for pairs in self.pairs.split(16384):
                a,b = pairs.T
                # 10 nm outward margin only affects candidate selection, not
                # the exact FCL acceptance distance or collision policy.
                masks.append(((low[:,a]-high[:,b] <= self.scene.clearance+1e-8) &
                              (low[:,b]-high[:,a] <= self.scene.clearance+1e-8)).all(dim=-1))
            near = torch.cat(masks,dim=1)
            self.refine_boxes(geoms,near)
            return poses.cpu().numpy(),near.cpu().numpy(),qs,grips

    def refine_boxes(self, geoms, near):
        """Reject only provably separated convex/box pairs on the GPU.

        Full face/edge SAT handles arbitrary rotations; AABBs alone cannot do
        this. The 10 nm outward bound only changes culling. Remaining distance
        and penetration queries preserve FCL's existing contact policy.
        """
        torch=self.torch
        for a,pair_ids,box_ids,vertices,normals,edges in self.sat_groups:
            # Compact on-device before allocating projection tensors. The
            # nonzero sync is per hull group, not per pose or collision pair.
            selected=torch.nonzero(near[:,pair_ids],as_tuple=False)
            if selected.numel()==0:continue
            for chunk in selected.split(128):
                row,column=chunk.T
                ta=geoms[row,a];tb=geoms[row,box_ids[column]]
                # Express box axes and center in the convex solid's frame.
                relative=ta[:,:3,:3].transpose(1,2)@tb[:,:3,:3]
                center=(ta[:,:3,:3].transpose(1,2)@(tb[:,:3,3]-ta[:,:3,3])[...,None]).squeeze(-1)
                box_axes=relative.transpose(1,2)
                cross=torch.linalg.cross(edges[None,:,None,:],box_axes[:,None,:,:],dim=-1).flatten(1,2)
                length=torch.linalg.vector_norm(cross,dim=-1,keepdim=True)
                cross=cross/length.clamp_min(1e-12)
                # Near-parallel cross axes are omitted conservatively.
                cross=torch.where(length>1e-10,cross,torch.zeros_like(cross))
                axes=torch.cat([normals[None].expand(len(row),-1,-1),box_axes,cross],dim=1)
                projection=vertices[None]@axes.transpose(1,2)
                midpoint=(center[:,None,:]*axes).sum(-1)
                local_axes=axes@relative
                radius=(local_axes.abs()*self.extents[box_ids[column]][:,None,:]).sum(-1)
                gap=torch.maximum(projection.amin(1)-(midpoint+radius),
                                  midpoint-radius-projection.amax(1)).amax(1)
                near[row,pair_ids[column]] &= gap<=self.scene.clearance+1e-8

    def apply(self, batch, index):
        poses, near, qs, grips = batch
        scene = self.scene
        # FCL's EPA penetration depth can change at degenerate contacts even
        # with sub-femtometer transform differences. Preserve the original CPU
        # transforms for exact narrow-phase policy equivalence.
        scene.update_transforms(qs[index],grips[index])
        candidates = scene.pairs[near[index]]
        scene.update_objects()
        scene.contacts(candidates)
        scene._contact_pose=(tuple(qs[index]),float(grips[index]))



def posed_samples(model, data, samples, *, batch_size=32, first_cpu=True, min_gpu_batch=8):
    """Yield original sample tuples after setting their collision pose.

    CPU mode preserves the existing streaming path. GPU allocation/runtime errors
    propagate (never interpret a failed computation as a collision-free scene).
    No pending batch is stored on the model, so early exit cannot leak poses.
    """
    from ..grasp.validation import set_pose
    mode = os.environ.get('HEI_COLLISION_ACCELERATOR','auto')
    if mode not in ('auto','cpu','cuda'):
        raise ValueError('HEI_COLLISION_ACCELERATOR must be auto, cpu or cuda')
    model.collision_accelerator = getattr(model,'collision_accelerator','cpu_fcl')
    iterator = iter(samples)
    # Opening preflight commonly fails at its very first pose. Check that pose
    # before allocating GPU buffers or evaluating later trajectory samples.
    if first_cpu:
        first = next(iterator,None)
        if first is None: return
        set_pose(model,data,first[2],first[3])
        yield first
    gpu = None
    checked = False
    while True:
        rows = list(islice(iterator,batch_size if mode != 'cpu' else 1))
        if not rows: return
        if not checked and len(rows) >= min_gpu_batch and getattr(model,'backend',None) == 'fcl' and mode != 'cpu':
            checked = True
            try:
                import torch
            except ImportError:
                if mode == 'cuda': raise
            else:
                if torch.cuda.is_available():
                    gpu = getattr(model,'_cuda_batch',None)
                    if gpu is None:
                        gpu = model._cuda_batch = CudaBatch(model)
                elif mode == 'cuda':
                    raise RuntimeError('CUDA collision acceleration requested but unavailable')
        if gpu is None:
            for row in rows:
                set_pose(model,data,row[2],row[3])
                yield row
        else:
            model.collision_accelerator = 'cuda_fk_aabb_sat+fcl'
            batch = gpu.compute([r[2] for r in rows],[r[3] for r in rows])
            for i,row in enumerate(rows):
                gpu.apply(batch,i)
                yield row
