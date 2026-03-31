from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # archivo maestro de logging
        Node(
            package='saves',
            executable='logging_manager',
            output='screen',
        ),

        # datos de sensores
        Node(
            package='saves',
            executable='data_recorder',
            output='screen',
        ),

        # datos de los filtros ekf
        Node(
            package='saves',
            executable='ekf_recorder',
            output='screen',
        ),

        # datos del modelo del carril
        Node(
            package='saves',
            executable='control_signals',
            output='screen',
        ),

        # imagenes de la cámara
        Node(
            package='saves',
            executable='perception_recorder',
            output='screen',
        ),

        # datos de la laptop
        Node(
            package='saves',
            executable='healthy_recorder',
            output='screen',
        )
    ])

"""


        Node(
            package='saves',
            executable='path_global_recorder',
            output='screen',
        ),
"""