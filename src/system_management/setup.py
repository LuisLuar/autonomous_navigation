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
            'gui = gui.management_gui:main',
            'rele_combiner = others.rele_combiner_node:main',
            'VC_supervisor =            safety_controller.voltage_current_supervisor:main',
            'esp32_supervisor =         safety_controller.esp32_supervisor:main',
            'components_supervisor =    safety_controller.components_supervisor:main',
            'master_supervisor =        safety_controller.master_supervisor:main',
            'microros_agent_supervisor= safety_controller.microros_agent_supervisor:main',
            'camera_supervisor =        safety_controller.camera_supervisor:main',
            'gps_supervisor =           safety_controller.gps_supervisor:main',
            'rplidar_supervisor =       safety_controller.rplidar_supervisor:main',
            'test_lazo_abierto =    test_code.test_lazo_abierto:main',
            'test_lazo_cerrado =    test_code.gui_lazo_cerrado:main',
            'bridge_serial_control =        others.bridge_serial_control:main',
            'unifier_control_comunication =        others.unifier_control_comunication:main',
            'bridge_serial_safety =        others.bridge_serial_safety:main',
            'unifier_safety_comunication =        others.unifier_safety_comunication:main',
        ],
    },
)
