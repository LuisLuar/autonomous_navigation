from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )

    # Rutas a los archivos YAML específicos
    jetson_limits_yaml = os.path.join(global_params_path, 'jetson_limits.yaml')
    battery_limits_yaml = os.path.join(global_params_path, 'battery_limits.yaml')

    return LaunchDescription([
        Node(
            package='system_management',
            executable='camera_supervisor',
            output='screen'
        ),
        
        Node(
            package='system_management',
            executable='master_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='system_health',
            name='jetson_health_monitor',
            parameters=[jetson_limits_yaml], # Aquí se carga el archivo YAML
            output='screen'
        ),

        Node(
            package='system_management',
            executable='VC_supervisor',
            name='supervisor_voltage',
            parameters=[battery_limits_yaml],
            output='screen'
        ),

        Node(
            package='system_management',
            executable='esp32_safety',
            output='screen',
        ),

        
        
    ])



"""
        

        Node(
            package='system_management',
            executable='gps_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='microros_agent_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='esp32_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='components_supervisor',
            output='screen'
        ),

        
        

"""
