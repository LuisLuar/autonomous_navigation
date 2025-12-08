from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_stack',
            executable='capture_frame',
            name='capture_frame',
            output='screen',
        ),
        Node(
            package='perception_stack',
            executable='segmenter_node',
            name='lane_segmenter',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='detector_node',
            name='object_detector',
            output='screen',
        ),

    ])


""""""