from glob import glob
from setuptools import setup

package_name = 'axis_hole_bringup'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
      maintainer='ros_robots', maintainer_email='user@example.com',
      description='Synthetic Cartesian assembly bringup', license='Apache License 2.0',
      entry_points={'console_scripts': [
          'mock_perception_node = axis_hole_bringup.mock_perception_node:main',
          'mock_plant_node = axis_hole_bringup.mock_plant_node:main',
          'demo_executor_node = axis_hole_bringup.demo_executor_node:main']})
