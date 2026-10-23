from pathlib import Path
import xml.etree.ElementTree as E

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1] / 'src'


def test_model_uris_interfaces_and_units():
    xacro = pytest.importorskip('xacro')
    robot = E.fromstring(xacro.process_file(str(ROOT / 'hei_description/urdf/hei.urdf.xacro')).toxml())
    joints = {j.get('name'): j for j in robot.findall('joint')}
    hardware = robot.find('ros2_control')
    assert hardware.findtext('hardware/plugin') == 'mock_components/GenericSystem'
    assert len(hardware.findall('joint')) == 21
    assert len(hardware.findall('joint/command_interface')) == 19
    for joint in hardware.findall('joint'):
        assert joint.get('name') in joints
        if 'R_finger' in joint.get('name'):
            assert joint.find('command_interface') is None
    assert float(joints['lift_joint'].find('limit').get('lower')) == -0.8
    for mesh in robot.findall('.//mesh'):
        prefix = 'package://hei_description/'
        assert mesh.get('filename').startswith(prefix)
        assert (ROOT / 'hei_description' / mesh.get('filename').removeprefix(prefix)).exists()


def test_controller_configuration_covers_every_command_interface():
    config = yaml.safe_load((ROOT / 'hei_control/config/controllers.yaml').read_text())
    assert config['/**/controller_manager']['ros__parameters']['enforce_command_limits']
    expected = {f'{side}_joint{i}' for side in ('a_right', 'b_left') for i in range(1, 7)}
    expected |= {'a_right_end_joint_L_finger', 'b_left_end_joint_L_finger', 'lift_joint',
                 'front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint'}
    assigned = []
    for name, node in config.items():
        if name.endswith('/controller_manager'):
            continue
        params = node['ros__parameters']
        assigned += params.get('joints', params.get('wheel_names', []))
    assert set(assigned) == expected
    assert len(assigned) == len(set(assigned))


@pytest.mark.parametrize('arm,prefix', [('right', 'a_right'), ('left', 'b_left')])
def test_damiao_arm_description_has_no_mock_or_commands_by_default(arm, prefix):
    xacro = pytest.importorskip('xacro')
    path = str(ROOT / 'hei_description/urdf/hei_damiao_arm.urdf.xacro')
    profile = str(ROOT / f'hei_bringup/config/{arm}_arm.yaml')
    robot = E.fromstring(xacro.process_file(path, mappings={'config_file': profile}).toxml())
    hardware = robot.find('ros2_control')
    assert hardware.findtext('hardware/plugin') == 'hei_hardware/DamiaoSystem'
    assert hardware.findtext("hardware/param[@name='read_only']") == 'true'
    assert [joint.get('name') for joint in hardware.findall('joint')] == [f'{prefix}_joint{i}' for i in range(1, 7)]
    assert hardware.findall('joint/command_interface') == []
    assert not robot.findall('.//param[@name="initial_value"]')
    writable = E.fromstring(xacro.process_file(path, mappings={
        'config_file': profile, 'read_only': 'false'}).toxml())
    assert len(writable.findall('ros2_control/joint/command_interface')) == 6
