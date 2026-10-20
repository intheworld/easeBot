from setuptools import find_packages, setup
setup(name="hei_robot_manager", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/hei_robot_manager"]),
                  ("share/hei_robot_manager", ["package.xml"])],
      package_data={"hei_robot_manager": ["web/*"]},
      install_requires=["setuptools"], zip_safe=False,
      maintainer="easeBot maintainers", maintainer_email="maintainers@example.invalid",
      description="HEI robot ROS 2 native hei_robot_manager", license="Apache-2.0",
      entry_points={"console_scripts": ['robot_manager = hei_robot_manager.node:main', 'joint_demo = hei_robot_manager.demo:main']})
