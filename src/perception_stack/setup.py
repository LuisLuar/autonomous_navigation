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
            'calibrate_camera = perception_stack.calibrate_camera_main',
            'detector_yolo = object_detection.detector_yolo:main',
            'object_fusion = object_detection.object_fusion:main',
            'visualizer_detector = object_detection.visualizer_detector:main',
            'vision_speed_limiter = object_detection.vision_speed_limiter:main',
            'segmenter_yolop = semantic_segmentation.segmenter_yolop:main',      
            'segmenter_node = semantic_segmentation.segmenter_node:main',          
            'visualizer_segmentation = semantic_segmentation.visualizer_segmentation:main',
            'lane_error = semantic_segmentation.lane_error:main',
            'ipm = semantic_segmentation.ipm:main',
            'lane_extraction = semantic_segmentation.lane_extraction:main',
            'bev_center_debug = semantic_segmentation.bev_center_debug:main',
            'tracker_center_ekf = semantic_segmentation.tracker_center_ekf:main',
            'tracker_model_ekf = semantic_segmentation.tracker_model_ekf:main',
            'lane_modeling = semantic_segmentation.lane_modeling:main',
            'ipm_lane_detection = semantic_segmentation.ipm_lane_detection:main',
            'unified_manual = semantic_segmentation.unified_manual:main',
        ],
    },
)
