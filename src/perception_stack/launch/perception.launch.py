from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_stack',
            executable='capture_frame',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='segmenter_yolop',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='detector_yolo',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='visualizer_detector',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='visualizer_segmentation',
            output='screen',
        ),

    ])


"""


"""