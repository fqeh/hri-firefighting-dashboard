from setuptools import setup
from glob import glob

setup(name='hri_reasoning', version='0.1.0', packages=['hri_reasoning'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/hri_reasoning']),
                  ('share/hri_reasoning', ['package.xml']),
                  ('share/hri_reasoning/config',glob('config/*.json'))],
      install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
      maintainer='HRI Project', maintainer_email='research@example.org',
      description='Belief-aware warnings over Gazebo observations', license='Apache-2.0',
      entry_points={'console_scripts': [
          'warning_decision = hri_reasoning.rescue_runtime:warning_main',
          'rescue_routing = hri_reasoning.rescue_runtime:route_main',
          'oracle_warning = hri_reasoning.oracle_runtime:main',
          'public_perception = hri_reasoning.tom_runtime:perception_main',
          'bayesian_tom = hri_reasoning.tom_runtime:inference_main',
          'forest_manager = hri_reasoning.forest_runtime:forest_main',
          'scripted_uav = hri_reasoning.forest_runtime:uav_main',
          'scenario_manager = hri_reasoning.runtime:scenario_main',
          'human_behavior = hri_reasoning.runtime:human_main',
          'belief_warning = hri_reasoning.runtime:inference_main',
          'event_logger = hri_reasoning.runtime:logger_main']})
