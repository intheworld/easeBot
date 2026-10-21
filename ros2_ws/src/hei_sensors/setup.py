from setuptools import find_packages, setup
setup(name="hei_sensors", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/hei_sensors"]),
                  ("share/hei_sensors", ["package.xml", "README.md"])],
      package_data={"hei_sensors": ["web/*"]},
      install_requires=["setuptools"], zip_safe=False,
      maintainer="easeBot maintainers", maintainer_email="maintainers@example.invalid",
      description="HEI robot ROS 2 native hei_sensors", license="Apache-2.0",
      entry_points={"console_scripts": ['cameras = hei_sensors.cameras:main']})
