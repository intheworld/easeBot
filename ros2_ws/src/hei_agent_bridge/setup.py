from setuptools import find_packages, setup

setup(
    name='hei_agent_bridge', version='0.1.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/hei_agent_bridge']),
                ('share/hei_agent_bridge', ['package.xml', 'README.md', 'sdk-revision.txt', 'requirements-mhs.lock'])],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='easeBot maintainers', maintainer_email='maintainers@example.invalid',
    description='Read-only HEI ROS 2 bridge to Agnes MHS/MOS', license='Apache-2.0',
    entry_points={'console_scripts': ['agent_bridge = hei_agent_bridge.node:main']},
)
