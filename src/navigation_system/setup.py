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
            'information_osm = path_planner.information_osm:main',
            'osm_limiter = local_planner.osm_limiter:main',
            'robust_controller = local_planner.robust_controller:main',
            'goal_reached = path_planner.goal_reached:main',
        ],
    },
)
