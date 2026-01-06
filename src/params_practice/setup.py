from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'params_practice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ton_nom',
    maintainer_email='ton_email@example.com',
    description='Parameters practice package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'basic_param_node = params_practice.basic_param_node:main',
        'dynamic_param_node = params_practice.dynamic_param_node:main',
        'described_param_node = params_practice.described_param_node:main',
        'battery_monitor_params = params_practice.battery_monitor_params:main',
        'sensor_manager = params_practice.sensor_manager:main',
    ],
},
)
