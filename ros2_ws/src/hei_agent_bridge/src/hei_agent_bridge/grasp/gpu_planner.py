"""cuRobo v0.7.8 IK proposals, independently checked using our URDF FK.

This is not collision certification. All callers retain the existing complete
trajectory collision validation before execution. No hardware imports.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .planner import RightArmPlanner


class CudaRightArmPlanner(RightArmPlanner):
    backend = 'curobo'

    def __init__(self, urdf):
        super().__init__(urdf)
        self.cpu_fallbacks = 0
        import torch
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
        self.tensor_args = TensorDeviceType()
        cfg = RobotConfig.from_basic(str(self.fk.path), self.fk.base_frame,
                                     self.fk.tool_frame, self.tensor_args)
        # Finite search bounds for the two software-unbounded joints. This is
        # a solver search window, not a new hardware/execution limit.
        lower = self.lower
        upper = np.where(np.isfinite(self.upper), self.upper, 2*np.pi)
        cfg.kinematics.get_joint_limits().position.copy_(
            torch.as_tensor(np.array([lower, upper]), **self.tensor_args.as_torch_dict()))
        self.ik = IKSolver(IKSolverConfig.load_from_robot_config(
            cfg, None, num_seeds=16, position_threshold=.005,
            rotation_threshold=np.deg2rad(3), self_collision_check=False,
            self_collision_opt=False, use_cuda_graph=True,
            tensor_args=self.tensor_args))
        if list(self.ik.joint_names) != self.fk.names:
            raise ValueError('GPU IK joint order differs from calibrated arm')

    def proposals(self, targets, seed):
        from curobo.types.math import Pose
        targets = np.asarray(targets)
        if len(targets) == 0:
            return np.empty((0, 8, 6))
        # Fixed batch shape permits CUDA graph replay and bounds GPU memory use.
        batch_size = 32
        batches = []
        for offset in range(0, len(targets), batch_size):
            chunk = targets[offset:offset+batch_size]
            padded = np.concatenate([chunk, np.repeat(chunk[-1:], batch_size-len(chunk), axis=0)])
            quat = Rotation.from_matrix(padded[:, :3, :3]).as_quat()[:, [3, 0, 1, 2]]
            pose = Pose(self.tensor_args.to_device(padded[:, :3, 3].copy()),
                        self.tensor_args.to_device(quat.copy()))
            # v0.7.8 get_seed consumes (batch, n_seeds, dof).
            seeds = self.tensor_args.to_device(np.broadcast_to(seed, (batch_size, 1, 6)).copy())
            result = self.ik.solve_batch(pose, seed_config=seeds, return_seeds=8)
            batches.append(result.solution.detach().cpu().numpy().reshape(batch_size, -1, 6)[:len(chunk)])
        return np.concatenate(batches)

    def choose(self, target, seed, candidates, strict=True):
        candidates=np.asarray(candidates,float)
        if candidates.ndim!=2 or candidates.shape[1]!=6:
            raise ValueError('GPU IK returned no pose within calibrated bounds and endpoint tolerance')
        valid=np.isfinite(candidates).all(axis=1)&(candidates>=self.lower-1e-6).all(axis=1)&(candidates<=self.upper+1e-6).all(axis=1)
        candidates=candidates[valid]
        if strict and len(candidates):
            actual=(self.fk.batch(candidates) if len(candidates)>1 and hasattr(self.fk,'batch')
                    else np.array([self.fk(q) for q in candidates]))
            position=np.linalg.norm(actual[:,:3,3]-target[:3,3],axis=1)
            relative=target[:3,:3].T@actual[:,:3,:3]
            sine=np.linalg.norm(np.stack([relative[:,2,1]-relative[:,1,2],relative[:,0,2]-relative[:,2,0],
                                           relative[:,1,0]-relative[:,0,1]],axis=1),axis=1)/2
            angle=np.arctan2(sine,(np.trace(relative,axis1=1,axis2=2)-1)/2)
            candidates=candidates[(position<=.005)&(angle<=np.deg2rad(3))]
        # Approximate transit samples need only finite bounded configurations;
        # their actual geometry is checked on the generated joint trajectory.
        if not len(candidates):raise ValueError('GPU IK returned no pose within calibrated bounds and endpoint tolerance')
        return candidates[np.argmin(np.linalg.norm(candidates-seed,axis=1))].copy()

    def solve(self, target, seed, *, require_pose_accuracy=True):
        from .transforms import validate_transforms
        target=validate_transforms(np.asarray(target)[None],'GPU IK target')[0]
        seed=np.asarray(seed,float)
        if seed.shape!=(6,) or not np.isfinite(seed).all():raise ValueError('IK needs six finite seed angles')
        return self.solve_proposed(target,seed,self.proposals([target],seed)[0],
                                   require_pose_accuracy=require_pose_accuracy)

    def solve_proposed(self,target,seed,candidates,*,require_pose_accuracy=True):
        """Reuse an existing GPU batch; never submit a rejected target twice."""
        try:
            return self.choose(target,seed,candidates,require_pose_accuracy)
        except ValueError:
            self.cpu_fallbacks += 1
            return super().solve(target,seed,require_pose_accuracy=require_pose_accuracy)

    def cartesian_path(self,start_q,goal,step_m=.005,*,strict_intermediate=True,strict_endpoint=True):
        import cv2
        from .transforms import validate_transforms
        goal=validate_transforms(np.asarray(goal)[None],'Cartesian target')[0]
        if not np.isfinite(step_m) or step_m<=0:raise ValueError('Cartesian step must be positive')
        start=self.fk(start_q)
        delta=cv2.Rodrigues(goal[:3,:3]@start[:3,:3].T)[0].ravel()
        distance=float(np.linalg.norm(goal[:3,3]-start[:3,3]))
        if not np.isfinite(distance) or distance/step_m>1024:raise ValueError('Cartesian path exceeds 1024 sample budget')
        count=max(1,int(np.ceil(distance/step_m)),int(np.ceil(np.linalg.norm(delta)/.05)))
        targets=[]
        for alpha in np.linspace(0,1,count+1)[1:]:
            t=np.eye(4);t[:3,3]=(1-alpha)*start[:3,3]+alpha*goal[:3,3]
            t[:3,:3]=cv2.Rodrigues(alpha*delta)[0]@start[:3,:3];targets.append(t)
        proposals=self.proposals(targets,start_q);path=[];q=np.asarray(start_q)
        for i,(target,candidates) in enumerate(zip(targets,proposals)):
            strict=strict_endpoint if i==count-1 else strict_intermediate
            try:
                next_q=self.choose(target,q,candidates,strict)
                if np.max(np.abs(next_q-q))>.2:raise ValueError('GPU IK branch change')
            except ValueError:
                self.cpu_fallbacks += 1
                next_q=RightArmPlanner.solve(self,target,q,require_pose_accuracy=strict)
            if np.max(np.abs(next_q-q))>.2:raise ValueError('IK branch jump in Cartesian path')
            path.append(next_q);q=next_q
        return path
