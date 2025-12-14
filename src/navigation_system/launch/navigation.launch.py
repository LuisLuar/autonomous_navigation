
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
    ])
