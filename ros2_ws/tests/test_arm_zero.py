import importlib.util
from pathlib import Path
import numpy as np

p=Path(__file__).parents[1]/'src/hei_bringup/scripts/arm_zero.py'
spec=importlib.util.spec_from_file_location('arm_zero',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def test_zero_path_has_exact_endpoint_and_vr_speed_and_step():
    start=[1.4,-3.1,-2.5,1.5,-1.2,2.8]
    samples=list(module.zero_samples(start));times=np.array([t for t,q in samples]);q=np.array([q for t,q in samples])
    np.testing.assert_allclose(q[0],start);np.testing.assert_allclose(q[-1],np.zeros(6),atol=1e-14)
    assert np.all(np.diff(times)>0)
    assert np.max(np.abs(np.diff(q,axis=0)))<=.015+1e-12
    assert np.all(np.abs(np.diff(q,axis=0))/np.diff(times)[:,None]<=np.array(module.VR_SPEEDS)+1e-12)

def test_already_zero_stays_zero():
    assert all(q==[0.]*6 for _,q in module.zero_samples([0.]*6))
