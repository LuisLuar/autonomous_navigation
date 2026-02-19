
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Lanza nodo controlador robusto
        Node(
            package='navigation_system',
            executable='robust_controller',
            output='screen',    
        ),

    ])

"""
        # Lanzar nodo de planificador global
        Node(
            package='navigation_system',
            executable='global_planner',
            output='screen',
        ),
        
        # Lanza nodo de monitor de objetivo alcanzado
        Node(
            package='navigation_system',
            executable='goal_reached',
            output='screen',    
        ),

        
        

        # Lanza nodo Matching
        Node(
            package='navigation_system',
            executable='matching',
            output='screen',    
        ),

        # lanza nodeo de extraccion de elementos OSM
        Node(
            package='navigation_system',
            executable='information_osm',
            output='screen',    
        ), 

        # lanzar nodoglobal path map to odom
        Node(
            package='navigation_system',
            executable='global_path_map_to_odom',
            output='screen',    
        ),    

        
        

"""