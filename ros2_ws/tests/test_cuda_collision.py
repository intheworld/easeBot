"""CPU streaming contract and optional CUDA parity with the full robot model."""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'src/hei_agent_bridge/src'))
pytest.importorskip('fcl')
from hei_agent_bridge.collision.fcl_scene import FCLScene
from hei_agent_bridge.collision.cuda_batch import CudaBatch,posed_samples
URDF=Path(__file__).parents[1]/'src/hei_description/urdf/hei_body.urdf'
SCENE=dict(table_top_z_m=-.1,left_joint_rad=[0,-.5,-.5,0,0,0],lift_joint_m=0.,left_finger_opening_m=.04,clearance_m=.003,scene_confirmed=True)
Q=np.array([0,-.5,-.5,0,0,0])


def test_cpu_stream_is_lazy_and_no_pending_pose(monkeypatch):
    monkeypatch.setenv('HEI_COLLISION_ACCELERATOR','cpu')
    model=FCLScene(URDF,{},SCENE)
    def rows():
        yield 0.,'first',Q,-2.
        raise AssertionError('Should stop reading at early collision')
    stream=posed_samples(model,model.data,rows())
    assert next(stream)[1]=='first'
    stream.close()
    other=Q+.1
    assert list(posed_samples(model,model.data,[(1.,'next',other,-3.)]))[0][1]=='next'
    expected=FCLScene(URDF,{},SCENE);expected.set_pose(other,-3.)
    np.testing.assert_allclose(model.geom_poses,expected.geom_poses)


def test_invalid_accelerator_rejected(monkeypatch):
    monkeypatch.setenv('HEI_COLLISION_ACCELERATOR','bad')
    model=FCLScene(URDF,{},SCENE)
    with pytest.raises(ValueError,match='ACCELERATOR'):
        list(posed_samples(model,model.data,[(0.,'test',Q,-3.)]))


def test_cuda_full_geometry_parity_and_early_exit(monkeypatch):
    torch=pytest.importorskip('torch')
    if not torch.cuda.is_available():pytest.skip('Requires CUDA')
    monkeypatch.setenv('HEI_COLLISION_ACCELERATOR','cuda')
    plan={'carry':{'status':'attached','payload':{'T_tcp_object':np.eye(4).tolist(),'size_m':[.06,.04,.03]}}}
    cpu=FCLScene(URDF,plan,SCENE);gpu=FCLScene(URDF,plan,SCENE)
    batcher=CudaBatch(gpu)
    rng=np.random.default_rng(17)
    qs=rng.uniform([-.2,-2,-2,-1,-1,-2],[1.4,.5,.5,1.4,1,2],(33,6))
    grips=rng.uniform(-4.5,0,33)
    batch=batcher.compute(qs,grips)
    for i in range(len(qs)):
        cpu.set_pose(qs[i],grips[i])
        np.testing.assert_allclose(batch[0][i],np.asarray([cpu.transforms[n] for n in cpu.names]),atol=1e-10,rtol=0)
        batcher.apply(batch,i)
        np.testing.assert_allclose(np.asarray(list(gpu.transforms.values())),np.asarray(list(cpu.transforms.values())),atol=1e-10,rtol=0)
        a={(c.geom1,c.geom2):c.dist for c in cpu.data.contact}
        b={(c.geom1,c.geom2):c.dist for c in gpu.data.contact}
        assert a.keys()==b.keys()
        np.testing.assert_allclose(list(a.values()),list(b.values()),atol=1e-8,rtol=0)
    stream=posed_samples(gpu,gpu.data,[(i,'test',q,g) for i,(q,g) in enumerate(zip(qs,grips))])
    next(stream);next(stream);stream.close()
    assert gpu.collision_accelerator=='cuda_fk_aabb_sat+fcl'
    list(posed_samples(gpu,gpu.data,[(0,'next',Q,-1.)]))
    cpu.set_pose(Q,-1.)
    np.testing.assert_allclose(np.asarray(list(gpu.transforms.values())),np.asarray(list(cpu.transforms.values())),atol=1e-10,rtol=0)
    with pytest.raises(ValueError,match='finite'):
        batcher.compute([[float('nan')]*6],[-2.])


def test_preflight_batch_includes_first_pose_and_reuses_contacts(monkeypatch):
    torch=pytest.importorskip('torch')
    if not torch.cuda.is_available():pytest.skip('Requires CUDA')
    monkeypatch.setenv('HEI_COLLISION_ACCELERATOR','cuda')
    model=FCLScene(URDF,{},SCENE)
    qs=[Q,Q+.01,Q+.02]
    rows=[(i,'start',q,-3.) for i,q in enumerate(qs)]
    for row in posed_samples(model,model.data,rows,first_cpu=False,min_gpu_batch=2):
        count=model.query_count
        model.set_pose(row[2],row[3])
        assert model.query_count==count
    assert model.collision_accelerator=='cuda_fk_aabb_sat+fcl'


def test_gpu_sat_rejects_rotated_box_aabb_false_positive():
    torch=pytest.importorskip('torch')
    if not torch.cuda.is_available():pytest.skip('Requires CUDA')
    from types import SimpleNamespace
    from scipy.spatial.transform import Rotation
    from hei_agent_bridge.collision.convex import box_polytope
    shape=box_polytope([.2,.01,.01]);a=np.eye(4);b=np.eye(4)
    a[:3,:3]=b[:3,:3]=Rotation.from_euler('z',45,degrees=True).as_matrix()
    b[:3,3]=a[:3,1]*.02
    def geom(t):return ('box','box',np.eye(4),None,np.array([-.1,-.005,-.005]),np.array([.1,.005,.005]),shape)
    scene=SimpleNamespace(transforms={'a':a,'b':b},names=['a','b'],geom_bodyid=[0,1],
        local=np.array([np.eye(4)]*2),centers=np.zeros((2,3)),extents=np.array([[.1,.005,.005]]*2),
        pairs=np.array([[0,1]]),geometries=[geom(a),geom(b)],joints=[],extra={},ids={'a':0,'b':1},clearance=0.)
    batch=CudaBatch(scene).compute(np.zeros((2,6)),np.zeros(2))
    assert not batch[1].any()
    # Positive clearance still admits the same near pair for exact FCL distance.
    scene.clearance=.011
    assert CudaBatch(scene).compute(np.zeros((2,6)),np.zeros(2))[1].all()
