from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        #Interfaz Grafica
        Node(
            package='system_management',
            executable='gui',
            output='screen'
        ),

        # Lanza la visualización en RVIZ2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_description'),
                    'launch',
                    'rviz2.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ), 

    ])

""" 
        #Nodo auxiliar para combinar bits de reles
        Node(
            package='system_management',
            executable='rele_combiner',
            output='screen'
        ),

"""