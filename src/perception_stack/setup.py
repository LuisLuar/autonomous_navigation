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
            'detector_yolo_right = object_detection.detector_yolo_right:main',
            'detector_yolo_left = object_detection.detector_yolo_left:main',
            'object_fusion = object_detection.object_fusion:main',
            'object_fusion_right = object_detection.object_fusion_right:main',
            'object_fusion_left = object_detection.object_fusion_left:main',
            'visualizer_detector = object_detection.visualizer_detector:main',
            'vision_speed_limiter = object_detection.vision_speed_limiter:main',
            'segmenter_yolop = semantic_segmentation.segmenter_yolop:main',               
            'visualizer_segmentation = semantic_segmentation.visualizer_segmentation:main',
            'lane_error = semantic_segmentation.lane_error:main',
            'ipm_lane_detection = semantic_segmentation.ipm_lane_detection:main',
            'pixel_to_meter_front = semantic_segmentation.pixel_to_meter_front:main',
            'pixel_to_meter_left = semantic_segmentation.pixel_to_meter_left:main', 
            'pixel_to_meter_right = semantic_segmentation.pixel_to_meter_right:main',
            'calibrate_ipm = semantic_segmentation.calibrate_ipm:main',
            'filtered = semantic_segmentation.filtered:main',
            'generate_target = semantic_segmentation.generate_target:main',
            'canny = semantic_segmentation.canny:main',
            'canny_left = semantic_segmentation.canny_left:main',
            'canny_right = semantic_segmentation.canny_right:main',
            'topology = semantic_segmentation.topology:main', 
            'lane_global_memory = semantic_segmentation.lane_global_memory:main', 
            'segmenter_yolop_left = semantic_segmentation.segmenter_yolop_left:main', 
            'segmenter_yolop_right = semantic_segmentation.segmenter_yolop_right:main', 
            'visualizer_camera_front = perception_stack.visualizer_camera_front:main',
            'visualizer_camera_right = perception_stack.visualizer_camera_right:main',
            'visualizer_camera_left = perception_stack.visualizer_camera_left:main',
            'object_to_odom = object_detection.object_to_odom:main',
            'correct_color_left = perception_stack.correct_color_left:main',
            'correct_color_right = perception_stack.correct_color_right:main',
            'lane_pointcloud_merger = semantic_segmentation.lane_pointcloud_merger:main',
            'detector_senaletica = object_detection.detector_senaletica:main',
            'lane_detector = semantic_segmentation.lane_detector:main',
            'base_footprint_topology = semantic_segmentation.base_footprint_topology:main',
            'lane_model_estimator = semantic_segmentation.lane_model_estimator:main',
            'lane_model_tracking = semantic_segmentation.lane_model_tracking:main',
            'inverse_IPM = perception_stack.inverse_IPM:main'
            
        ],
    },
)
