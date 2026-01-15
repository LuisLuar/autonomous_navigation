from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_management',
            executable='camera_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='gps_supervisor',
            output='screen'
        ),
        
        Node(
            package='system_management',
            executable='master_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='microros_agent_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='rplidar_supervisor',
            output='screen'
        ),


        Node(
            package='system_management',
            executable='system_health',
            output='screen'
        ),
        
    ])

#Node(
#            package='system_management',
#            executable='esp32_supervisor',
#            output='screen'
#        ),

"""
        Node(
            package='system_management',
            executable='components_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='VC_supervisor',
            output='screen'
        ),
        

"""
