from glob import glob
from setuptools import setup

setup(name='hri_bringup', version='0.1.0', packages=['hri_bringup'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/hri_bringup']),
                  ('share/hri_bringup', ['package.xml']),
                  ('share/hri_bringup/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      tests_require=['pytest'],
      maintainer='HRI Project', maintainer_email='research@example.invalid',
      description='Gazebo/ROS integration and validation', license='Apache-2.0',
      entry_points={'console_scripts': [
        'gazebo_process = hri_bringup.gazebo_process:main',
          'verify_telemetry = hri_bringup.verify_telemetry:main',
          'operator_warning = hri_bringup.operator_warning:main',
          'mission_monitor = hri_bringup.mission_monitor:main',
          'safety_controller = hri_bringup.safety_controller:main',
          'waypoint_mission = hri_bringup.waypoint_mission:main',
          'operator_navigation = hri_bringup.operator_navigation:main']})
