from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception_stack'

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
            'detector_node = object_detection.detector_node:main',
            'segmenter_node = semantic_segmentation.segmenter_node:main',
            'capture_frame = perception_stack.capture_frame:main',  
            'lane_detector = semantic_segmentation.lane_detector:main',
            'visualization_segmentation = semantic_segmentation.visualization_segmentation:main',
        ],
    },
)
