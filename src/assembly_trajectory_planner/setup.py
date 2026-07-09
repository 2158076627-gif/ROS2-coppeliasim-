from glob import glob
import os

from setuptools import setup


package_name = 'assembly_trajectory_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_robots',
    maintainer_email='user@example.com',
    description='Assembly trajectory planner for perception-guided insertion tasks',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_planner_node = assembly_trajectory_planner.trajectory_planner_node:main',
        ],
    },
)
