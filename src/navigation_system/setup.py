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
            'cmd_vel_fusion = local_planner.cmd_vel_fusion:main',
            'lidar_speed_limiter = local_planner.lidar_speed_limiter:main',
            'lidar_lateral_bias = local_planner.lidar_lateral_bias:main',
            'bt_manager = behavior_tree.bt_manager:main',
            'map_matching = path_planner.map_matching:main',
            'matching = path_planner.matching:main',
            'lane_local_path_extractor = local_planner.lane_local_path_extractor:main',
            'pursuit_controller = local_planner.pursuit_controller:main',
            'brain_controller = local_planner.brain_controller:main',
            'information_osm = path_planner.information_osm:main',
            'osm_limiter = local_planner.osm_limiter:main',
        ],
    },
)
