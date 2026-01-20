
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        
        # Lanzar nodo de planificador global
        Node(
            package='navigation_system',
            executable='global_planner',
            output='screen',
        ),

        # Lanzar nodo de limitador de velocidad LIDAR
        Node(
            package='navigation_system',
            executable='lidar_speed_limiter',
            output='screen',    
        ),

        # Lanzar nodo de sesgo lateral LIDAR
        Node(
            package='navigation_system',
            executable='lidar_lateral_bias',
            output='screen',    
        ),

        # Lanza nodo de monitor de objetivo alcanzado
        Node(
            package='navigation_system',
            executable='goal_reached',
            output='screen',    
        ),

        # Lanza nodo controlador robusto
        Node(
            package='navigation_system',
            executable='robust_controller',
            output='screen',    
        ), 

        # lanzar nodoglobal path map to odom
        Node(
            package='navigation_system',
            executable='global_path_map_to_odom',
            output='screen',    
        ),         
        
    ])

"""
        #Lanzar nodo de seguidor de path local
        Node(
            package='navigation_system',
            executable='local_path_follower',
            output='screen',    
        ),

        #Lanza nodo de map matching
        Node(
            package='navigation_system',
            executable='map_matching',
            output='screen',    
        ),

        #Lanzar nodo de extracción de path local desde línea de ego
        Node(
            package='navigation_system',
            executable='lane_local_path_extractor',
            output='screen',    
        ),

        #Lanzar nodo de controlador pursuit Stanley
        Node(
            package='navigation_system',
            executable='pursuit_controller',
            output='screen',    
        ),

        #Lanzar nodo de controlador cerebral
        Node(
            package='navigation_system',
            executable='brain_controller',
            output='screen',    
        ),

        # Lanza nodo Matching
        Node(
            package='navigation_system',
            executable='matching',
            output='screen',    
        ),

        # Lanzar nodo de fusión de velocidades
        Node(
            package='navigation_system',
            executable='cmd_vel_fusion',
            output='screen',    
        ),

        # lanza nodeo de extraccion de elementos OSM
        Node(
            package='navigation_system',
            executable='information_osm',
            output='screen',    
        ),    

        
        

"""