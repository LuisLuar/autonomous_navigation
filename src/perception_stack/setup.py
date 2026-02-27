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
            'capture_frame = perception_stack.capture_frame:main',
            'calibrate_camera = perception_stack.calibrate_camera:main',
            'detector_yolo = object_detection.detector_yolo:main',        
            'object_fusion = object_detection.object_fusion:main',        
            'vision_speed_limiter = object_detection.vision_speed_limiter:main',            
            'calibrate_ipm = semantic_segmentation.calibrate_ipm:main',
            'visualizer_camera_front = perception_stack.visualizer_camera_front:main',            
            'object_to_odom = object_detection.object_to_odom:main',
            'detector_senaletica = object_detection.detector_senaletica:main',
            'lane_model_estimator = semantic_segmentation.lane_model_estimator:main',
            'lane_model_tracking = semantic_segmentation.lane_model_tracking:main',
            'inverse_IPM = perception_stack.inverse_IPM:main'
            
        ],
    },
)
