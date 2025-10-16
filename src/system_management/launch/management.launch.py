from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_management',
            executable='rele_combiner',
            name='rele_combiner_node',
            output='screen'
        )
    ])