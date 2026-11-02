"""Independent SAT vs native FCL, including the actual rejected finger/wall pose."""
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from hei_agent_bridge.collision.convex import box_polytope,separating_gap
pytest.importorskip('fcl')
from hei_agent_bridge.collision.fcl_scene import FCLScene
from test_online_collision import URDF,SCENE


def test_rotated_thin_boxes_separate_despite_overlapping_world_aabbs():
    shape=box_polytope([.2,.01,.01]);a=np.eye(4);b=np.eye(4)
    a[:3,:3]=b[:3,:3]=Rotation.from_euler('z',45,degrees=True).as_matrix()
    b[:3,3]=a[:3,1]*.02
    assert separating_gap(shape,a,shape,b)['gap_m']==pytest.approx(.01,abs=1e-12)


def test_containment_uses_translation_to_separate_not_interval_intersection():
    a=box_polytope([.2]*3);b=box_polytope([.02]*3)
    assert separating_gap(a,np.eye(4),b,np.eye(4))['gap_m']==pytest.approx(-.11)


def test_independent_sat_agrees_with_fcl_on_transformed_robot_finger():
    import fcl
    model=FCLScene(URDF,{},SCENE)
    i=next(i for i,g in enumerate(model.geometries) if g[1]=='a_right_end_link_L_finger_collision0_part0')
    shape=model.geometries[i][6];native=model.geometries[i][3]
    box=box_polytope([.03,.04,.05]);rng=np.random.default_rng(46)
    a=fcl.CollisionObject(native);b=fcl.CollisionObject(fcl.Box(.03,.04,.05))
    for _ in range(60):
        ta=np.eye(4);tb=np.eye(4)
        ta[:3,:3]=Rotation.random(random_state=rng).as_matrix()
        tb[:3,:3]=Rotation.random(random_state=rng).as_matrix()
        tb[:3,3]=ta[:3,:3]@shape['vertices'].mean(0)+rng.uniform(-.06,.06,3)
        a.setTransform(fcl.Transform(ta[:3,:3].copy(),ta[:3,3].copy()))
        b.setTransform(fcl.Transform(tb[:3,:3].copy(),tb[:3,3].copy()))
        result=fcl.CollisionResult()
        fcl.collide(a,b,fcl.CollisionRequest(enable_contact=True),result)
        gap=separating_gap(shape,ta,box,tb)['gap_m']
        assert result.is_collision==(gap<=0)


def test_saved_release_wall_contact_matches_independent_sat():
    import json
    f=json.loads((Path(__file__).parent/'fixtures/finger_wall_contact.json').read_text())
    model=FCLScene(URDF,{'carry':{'obstacles':[f['obstacle']]}},f['scene'])
    model.set_pose(f['q'],f['grip'])
    contacts=[c for c in model.data.contact if model.geometries[c.geom1][0]=='placed_'+f['obstacle']['id']
              and model.geometries[c.geom2][1]=='a_right_end_link_L_finger_collision0_part0']
    assert len(contacts)==1
    c=contacts[0];a,b=c.geom1,c.geom2
    audit=separating_gap(model.geometries[a][6],model.geom_poses[a],
                         model.geometries[b][6],model.geom_poses[b])
    assert -c.dist==pytest.approx(f['expected_depth_m'],abs=1e-9)
    assert audit['gap_m']==pytest.approx(c.dist,abs=1e-9)
