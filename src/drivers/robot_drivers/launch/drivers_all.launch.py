from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lanzar RPLidar A1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_a1_launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),
        
        # Lanzar Asus Xtion
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_drivers'),
                    'launch',
                    'asus_xtion.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),
        
        # Lanzar nodo GPS
        Node(
            package='robot_drivers',
            executable='gps',
            output='screen',
            name='gps_node'
        ),

        # Lanzar WebSocket Bridge
        Node(
            package='robot_drivers',
            executable='websocket_bridge',
            output='screen',
            name='websocket_ros_bridge'
        ),
    ])