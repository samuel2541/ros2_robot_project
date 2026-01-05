from setuptools import find_packages, setup

package_name = 'Tp4_exercice2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='samuel@todo.todo',
    description='TP4 Exercice 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'config_server = Tp4_exercice2.config_server:main',
            'config_client = Tp4_exercice2.config_client:main',
        ],
    },
)
