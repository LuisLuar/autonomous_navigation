from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_management',
            executable='bridge_serial_control',
            name='bridge_serial_control',
            output='screen'
        ),
        Node(
            package='system_management',
            executable='unifier_control_comunication',
            name='unifier_control_comunication',
            output='screen'
        ),
        
    ])

"""

    Node(
            package='system_management',
            executable='bridge_serial_safety',
            name='bridge_serial_safety',
            output='screen'
        ),
        Node(
            package='system_management',
            executable='unifier_safety_comunication',
            name='unifier_safety_comunication',
            output='screen'
        ),

"""