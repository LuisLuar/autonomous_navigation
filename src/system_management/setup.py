from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'system_management'

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
            'management_gui = gui.management_gui:main',
            'system_monitor = system_monitor.monitor_node:main',
            'VC_supervisor = safety_controller.voltage_current_supervisor:main',
            'rele_combiner = safety_controller.rele_combiner_node:main',
            'esp32_supervisor = safety_controller.esp32_supervisor:main',
            'components_supervisor = safety_controller.components_supervisor:main',
            'gui = gui.management_gui:main',
            'microros_agent_supervisor = safety_controller.microros_agent_supervisor:main',
        ],
    },
)
