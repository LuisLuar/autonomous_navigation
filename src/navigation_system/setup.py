from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navigation_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raynel',
    maintainer_email='luchitov2001@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Aquí registrarás tus ejecutables
            'global_planner = path_planner.global_planner:main',
            'local_planner = path_planner.local_planner:main',
            'bt_manager = behavior_tree.bt_manager:main',
        ],
    },
)
