from setuptools import find_packages, setup
from glob import glob

setup(
    name='hei_agent_bridge', version='0.1.0',
    packages=find_packages(where='src'), package_dir={'': 'src'},
    data_files=[('share/ament_index/resource_index/packages', ['resource/hei_agent_bridge']),
                ('share/hei_agent_bridge', ['package.xml', 'README.md', 'sdk-revision.txt', 'requirements-mhs.lock']),
                ('share/hei_agent_bridge/config', glob('config/*.json') + glob('config/*.npz'))],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='easeBot maintainers', maintainer_email='maintainers@example.invalid',
    description='HEI ROS 2 observation and explicit skill set bridge to Agnes MHS/MOS', license='Apache-2.0',
    entry_points={'console_scripts': ['agent_bridge = hei_agent_bridge.node:main']},
)
