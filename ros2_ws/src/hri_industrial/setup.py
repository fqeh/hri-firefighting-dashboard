from glob import glob
from setuptools import setup

setup(name='hri_industrial',version='0.1.0',packages=['hri_industrial'],
    data_files=[('share/ament_index/resource_index/packages',['resource/hri_industrial']),
        ('share/hri_industrial',['package.xml']),
        ('share/hri_industrial/config',glob('config/*.json')),
        ('share/hri_industrial/worlds',glob('worlds/*.sdf')),
        ('share/hri_industrial/models/oil_robot',glob('models/oil_robot/*')),
        ('share/hri_industrial/models/tank_damage_visual',glob('models/tank_damage_visual/*')),
        ('share/hri_industrial/config/scenarios',glob('config/scenarios/*.json'))],
    install_requires=['setuptools'],tests_require=['pytest'],zip_safe=True,
    maintainer='HRI Project',maintainer_email='research@example.invalid',
    description='Fictional oil-facility scene and shared-autonomy adapters',license='Apache-2.0',
    entry_points={'console_scripts':[
        'industrial_manager=hri_industrial.incidents:main',
        'industrial_perception=hri_industrial.perception:main',
        'industrial_routing=hri_industrial.routing:main',
        'operator_input=hri_industrial.operator_input:main',
        'operator_tom=hri_industrial.operator_inference:main',
        'operator_warning_policy=hri_industrial.operator_inference:warning_main',
        'operator_oracle=hri_industrial.oracle:main',
        'shared_navigation=hri_industrial.shared_navigation:main',
        'simulated_operator=hri_industrial.simulated_operator:main']})
