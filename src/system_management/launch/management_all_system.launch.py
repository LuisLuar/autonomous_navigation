from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Micro ROS Agent como Node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen',
            name='micro_ros_agent'
        ),

        # Lanzar URDF y TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_description'),
                    'launch',
                    'robot_state_publisher.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Lanzar camara - gps -lidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_drivers'),
                    'launch',
                    'drivers_all.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Lanzar el filtro EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('filter_ekf'),
                    'launch',
                    'dual_ekf_navsat.launch.py'
                ])
            ]),
            launch_arguments={}.items()
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

        # Lanza los nodos de safety
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('system_management'),
                    'launch',
                    'status.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Lanza los nodos de GUI y rele combiner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('system_management'),
                    'launch',
                    'others.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),   
    ])