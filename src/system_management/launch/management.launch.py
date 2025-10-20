from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_management',
            executable='rele_combiner',
            name='rele_combiner_node',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='esp32_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='VC_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='gui',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='microros_agent_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='components_supervisor',
            output='screen'
        ),

    ])