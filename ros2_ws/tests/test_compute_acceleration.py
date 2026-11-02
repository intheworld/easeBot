"""GPU selection and independent FK/pose acceptance; no robot or GPU required."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
import pytest
from hei_agent_bridge.grasp.compute_device import inference_device
from hei_agent_bridge.grasp.kinematics import RightArmFK
from hei_agent_bridge.grasp.gpu_planner import CudaRightArmPlanner
from hei_agent_bridge.grasp.planner import RightArmPlanner

URDF=Path(__file__).parents[1]/'src/hei_description/urdf/hei_body.urdf'

@pytest.mark.parametrize('available,expected',[(True,'cuda:0'),(False,'cpu')])
def test_device_selection(monkeypatch,available,expected):
    inference_device.cache_clear()
    monkeypatch.setitem(sys.modules,'torch',SimpleNamespace(cuda=SimpleNamespace(is_available=lambda:available)))
    assert inference_device()==expected
    inference_device.cache_clear()


def test_gpu_proposals_rechecked_by_cpu_fk():
    planner=CudaRightArmPlanner.__new__(CudaRightArmPlanner)
    RightArmPlanner.__init__(planner,URDF)
    q=np.array([.3,-1.,-1.,.5,.1,.2]);target=planner.fk(q)
    actual=planner.choose(target,q,np.array([q+1.,q]))
    np.testing.assert_allclose(actual,q)
    with pytest.raises(ValueError,match='no pose'):
        planner.choose(target,q,np.array([q+1.,np.full(6,np.nan)]))


def test_cached_fk_matches_xml_reference():
    import cv2
    fk=RightArmFK(URDF)
    for q in np.random.default_rng(4).normal(size=(20,6)):
        t=np.eye(4);i=0
        for j in fk.chain:
            origin=j.find('origin');a=np.eye(4)
            if origin is not None:
                a[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                r,p,y=np.fromstring(origin.get('rpy','0 0 0'),sep=' ')
                a[:3,:3]=cv2.Rodrigues(np.array([0.,0.,y]))[0]@cv2.Rodrigues(np.array([0.,p,0.]))[0]@cv2.Rodrigues(np.array([r,0.,0.]))[0]
            t=t@a
            if j.get('type')=='revolute':
                b=np.eye(4);b[:3,:3]=cv2.Rodrigues(np.fromstring(j.find('axis').get('xyz'),sep=' ')*q[i])[0];t=t@b;i+=1
        np.testing.assert_allclose(fk(q),t,atol=1e-14)


def test_common_path_reused_without_exposing_cached_mutable_arrays():
    from hei_agent_bridge.grasp.planner_backend import cached_cartesian_path
    calls=[]
    planner=SimpleNamespace(cartesian_path=lambda *a,**k:(calls.append(1) or [np.ones(6)]))
    first=cached_cartesian_path(planner,np.zeros(6),np.eye(4));first[0][0]=999
    second=cached_cartesian_path(planner,np.zeros(6),np.eye(4))
    assert len(calls)==1 and second[0][0]==1
    cached_cartesian_path(planner,np.zeros(6),np.eye(4),strict_intermediate=False)
    assert len(calls)==2


def test_worker_configuration_keeps_virtualenv_executable_path(tmp_path):
    import os
    from hei_agent_bridge.arm.service import prepare_config
    python=tmp_path/'venv/bin/python';python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    cfg=prepare_config({'worker_python':str(python),'joint_speed_rad_s':.12})
    assert cfg['worker_python']==str(python)
    assert os.access(cfg['worker_python'],os.X_OK)


def test_collision_mesh_cache_invalidates_on_file_change(tmp_path):
    import struct
    from hei_agent_bridge.grasp.validation import stl_components
    path=tmp_path/'tetra.stl'
    def write(scale):
        vertices=np.array([[0,0,0],[1,0,0],[0,1,0],[0,0,1]],float)*scale
        faces=[(0,1,2),(0,1,3),(0,2,3),(1,2,3)]
        path.write_bytes(b' '*80+struct.pack('<I',4)+b''.join(
            struct.pack('<12fH',0,0,0,*vertices[list(face)].ravel(),0) for face in faces))
    write(1)
    a=stl_components(path);b=stl_components(path)
    assert a is b and not a[0].flags.writeable
    write(2)
    c=stl_components(path)
    assert c is not a and c[0].max()==2


def test_rejected_gpu_batch_is_not_submitted_again_before_cpu_fallback(monkeypatch):
    planner=CudaRightArmPlanner.__new__(CudaRightArmPlanner)
    RightArmPlanner.__init__(planner,URDF);planner.cpu_fallbacks=0
    q=np.array([.3,-1.,-1.,.5,.1,.2]);target=planner.fk(q)
    planner.proposals=lambda *args:(_ for _ in ()).throw(AssertionError('Duplicate GPU submission'))
    calls=[]
    monkeypatch.setattr(RightArmPlanner,'solve',lambda self,target,seed,**kw:
                        (calls.append(seed.copy()) or q.copy()))
    result=planner.solve_proposed(target,q,np.array([q+1]))
    np.testing.assert_allclose(result,q)
    assert len(calls)==1 and planner.cpu_fallbacks==1
