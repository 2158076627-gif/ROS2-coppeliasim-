from glob import glob
import os

from setuptools import setup


package_name = 'compliant_servo_control'

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
    description='Impedance and admittance servo control for insertion assembly',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compliant_servo_node = compliant_servo_control.compliant_servo_node:main',
            'mock_force_sensor_node = compliant_servo_control.mock_force_sensor_node:main',
        ],
    },
)
