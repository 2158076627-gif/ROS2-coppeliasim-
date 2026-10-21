from glob import glob
from setuptools import setup

package_name = 'coppeliasim_contact'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                  ('share/'+package_name, ['package.xml']),
                  ('share/'+package_name+'/launch', glob('launch/*.launch.py')),
                  ('share/'+package_name+'/config', glob('config/*'))],
      install_requires=['setuptools'], extras_require={'sim': ['coppeliasim-zmqremoteapi-client==2.0.4'],
                                                      'test': ['pytest'], 'report': ['matplotlib>=3.6']},
      zip_safe=True, maintainer='ros_robots', maintainer_email='user@example.com',
      description='Independent CoppeliaSim contact test rig adapter', license='Apache License 2.0',
      entry_points={'console_scripts': [
          'coppeliasim_bridge = coppeliasim_contact.bridge:main',
          'build_contact_scene = coppeliasim_contact.scene:main',
          'contact_experiment = coppeliasim_contact.experiments:main',
          'contact_acceptance = coppeliasim_contact.acceptance_cli:main']})
