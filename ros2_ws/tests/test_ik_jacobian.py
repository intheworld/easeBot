"""Independent central differences and finite-difference solver comparison."""
from pathlib import Path
import cv2
import numpy as np
import pytest
from scipy.optimize import least_squares
from hei_agent_bridge.grasp.planner import RightArmPlanner

URDF=Path(__file__).parents[1]/'src/hei_description/urdf/hei_body.urdf'


def residual(fk,q,target):
    actual=fk(q)
    return np.r_[actual[:3,3]-target[:3,3],.15*cv2.Rodrigues(target[:3,:3].T@actual[:3,:3])[0].ravel()]


def test_analytic_pose_jacobian_matches_central_differences():
    planner=RightArmPlanner(URDF);rng=np.random.default_rng(19)
    for _ in range(40):
        q=rng.uniform([-.1,-2.8,-2.8,-1,-1,-2],[1.3,-.1,-.1,1.3,1,2])
        target=planner.fk(q+rng.uniform(-.5,.5,6))
        value,jac=planner.pose_residual_jacobian(q,target)
        np.testing.assert_allclose(value,residual(planner.fk,q,target),atol=1e-12)
        numeric=np.column_stack([(residual(planner.fk,q+np.eye(6)[i]*1e-6,target)-
                                  residual(planner.fk,q-np.eye(6)[i]*1e-6,target))/2e-6 for i in range(6)])
        np.testing.assert_allclose(jac,numeric,atol=2e-8,rtol=1e-6)
        np.testing.assert_allclose(planner.fk.with_jacobian(q)[0],planner.fk(q),atol=1e-14)


@pytest.mark.parametrize('angle',[0.,1e-4,179.])
def test_orientation_jacobian_at_zero_and_large_errors(angle):
    planner=RightArmPlanner(URDF);q=np.array([.3,-1,-1,.5,.1,.2]);target=planner.fk(q)
    target[:3,:3]=target[:3,:3]@cv2.Rodrigues(np.array([0.,0.,np.deg2rad(angle)]))[0]
    _,jac=planner.pose_residual_jacobian(q,target)
    # OpenCV snaps tiny log vectors near zero; use a finite step above its cutoff.
    step=1e-4
    numeric=np.column_stack([(residual(planner.fk,q+np.eye(6)[i]*step,target)-
                              residual(planner.fk,q-np.eye(6)[i]*step,target))/2/step for i in range(6)])
    np.testing.assert_allclose(jac,numeric,atol=3e-5,rtol=3e-4)


def test_analytic_solver_preserves_accuracy_and_failure_decisions():
    planner=RightArmPlanner(URDF);rng=np.random.default_rng(47)
    for i in range(12):
        seed=np.array([.3,-1.,-1.,.5,.1,.2])+rng.uniform(-.1,.1,6)
        goal=planner.fk(seed+rng.uniform(-.1,.1,6))
        if i>=8:goal[:3,3]+=np.array([1.,1.,1.])
        reference=least_squares(lambda q:residual(planner.fk,q,goal),np.clip(seed,planner.lower,planner.upper),
            bounds=(planner.lower,planner.upper),max_nfev=250,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        actual=planner.fk(reference.x)
        accepted=np.linalg.norm(actual[:3,3]-goal[:3,3])<=.005 and np.linalg.norm(cv2.Rodrigues(goal[:3,:3].T@actual[:3,:3])[0])<=np.deg2rad(3)
        if not accepted:
            with pytest.raises(ValueError,match='IK failed'):planner.solve(goal,seed)
        else:
            result=planner.solve(goal,seed)
            error=residual(planner.fk,result,goal)
            assert np.linalg.norm(error[:3])<=.005 and np.linalg.norm(error[3:])<=.15*np.deg2rad(3)


def test_batched_fk_matches_scalar_urdf_fk():
    planner=RightArmPlanner(URDF);qs=np.random.default_rng(51).uniform(-3,3,(100,6))
    np.testing.assert_allclose(planner.fk.batch(qs),np.array([planner.fk(q) for q in qs]),atol=2e-14,rtol=0)
    assert planner.fk.batch(np.empty((0,6))).shape==(0,4,4)
    with pytest.raises(ValueError,match='finite'):planner.fk.batch(np.full((2,6),np.nan))


def test_vectorized_proposal_acceptance_matches_independent_scalar_check():
    from hei_agent_bridge.grasp.gpu_planner import CudaRightArmPlanner
    planner=CudaRightArmPlanner.__new__(CudaRightArmPlanner);RightArmPlanner.__init__(planner,URDF)
    rng=np.random.default_rng(23);seed=np.array([.3,-1,-1,.5,.1,.2]);target=planner.fk(seed)
    for strict in [True,False]:
        for i in range(30):
            candidates=seed+rng.normal(0,.01 if i<15 else .5,(8,6))
            if i%3==0:candidates[0,0]=np.nan
            valid=[]
            for q in candidates:
                if not np.isfinite(q).all() or np.any(q<planner.lower-1e-6) or np.any(q>planner.upper+1e-6):continue
                pose=planner.fk(q)
                if strict and (np.linalg.norm(pose[:3,3]-target[:3,3])>.005 or
                               np.linalg.norm(cv2.Rodrigues(target[:3,:3].T@pose[:3,:3])[0])>np.deg2rad(3)):continue
                valid.append((np.linalg.norm(q-seed),q))
            if not valid:
                with pytest.raises(ValueError,match='no pose'):planner.choose(target,seed,candidates,strict)
            else:np.testing.assert_array_equal(planner.choose(target,seed,candidates,strict),min(valid,key=lambda x:x[0])[1])


def test_old_resident_fk_class_still_works_during_deployment():
    from hei_agent_bridge.grasp.gpu_planner import CudaRightArmPlanner
    planner=CudaRightArmPlanner.__new__(CudaRightArmPlanner);RightArmPlanner.__init__(planner,URDF)
    fk=planner.fk;planner.fk=lambda q:fk(q)
    q=np.array([.3,-1,-1,.5,.1,.2]);target=fk(q)
    np.testing.assert_array_equal(planner.choose(target,q,np.array([q,q+.01])),q)
    np.testing.assert_allclose(RightArmPlanner.solve(planner,target,q),q,atol=1e-9)
