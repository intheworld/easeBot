import numpy as np
import pytest
from hei_agent_bridge.grasp.container_space import measured_floor


def test_rejected_tilt_preserves_diagnostics():
    x,y=np.meshgrid(np.linspace(-.05,.05,80),np.linspace(-.05,.05,80))
    points=np.c_[x.ravel(),y.ravel(),(.85+.12*x).ravel()]
    diagnostics={}
    with pytest.raises(ValueError,match='Support plane rejected: tilt='):
        measured_floor(points,diagnostics)
    report=diagnostics['support_plane']
    assert report['tilt_exceeded']
    assert not report['noise_exceeded']
    assert report['tilt_deg']>report['max_tilt_deg']


def test_flat_floor_keeps_acceptance_and_reports_metrics():
    x,y=np.meshgrid(np.linspace(-.05,.05,80),np.linspace(-.05,.05,80))
    points=np.c_[x.ravel(),y.ravel(),np.full(x.size,.85)]
    diagnostics={}
    height,mask=measured_floor(points,diagnostics)
    assert height==.85 and mask.all()
    assert diagnostics['support_plane']['tilt_deg']==0


def test_mild_tilt_is_allowed():
    x,y=np.meshgrid(np.linspace(-.04,.04,80),np.linspace(-.04,.04,80))
    points=np.c_[x.ravel(),y.ravel(),(.85+np.tan(np.deg2rad(3.1))*x).ravel()]
    diagnostics={}
    measured_floor(points,diagnostics)
    assert not diagnostics['support_plane']['tilt_exceeded']
    assert diagnostics['support_plane']['max_tilt_deg']==5.0
