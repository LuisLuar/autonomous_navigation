from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Nodo auxiliar para combinar bits de reles
        Node(
            package='system_management',
            executable='rele_combiner',
            output='screen'
        ),

        #Interfaz Grafica
        Node(
            package='system_management',
            executable='gui',
            output='screen'
        ),

    ])