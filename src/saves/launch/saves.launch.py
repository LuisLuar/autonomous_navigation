from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='saves',
            executable='logging_manager',
            output='screen',
        ),

        Node(
            package='saves',
            executable='data_recorder',
            output='screen',
        ),

        Node(
            package='saves',
            executable='ekf_recorder',
            output='screen',
        ),

        Node(
            package='saves',
            executable='path_global_recorder',
            output='screen',
        ),

        Node(
            package='saves',
            executable='params_cmd_vel_recorder',
            output='screen',
        ),

        Node(
            package='saves',
            executable='perception_recorder',
            output='screen',
        ),
        
        Node(
            package='saves',
            executable='healthy_recorder',
            output='screen',
        )
    ])