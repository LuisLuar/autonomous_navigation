import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Rutas a archivos de parámetros (ajusta si es necesario)
    calibration_file = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_front.json"

    return LaunchDescription([
        # 1. CAMERA NODE (Captura y Redimensión)
        Node(
            package='camera_cpp_nodes',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device': 2}],
            output='screen'
        ),

        # 2. SEGMENTER NODE (YOLOPv2 Inferencia)
        Node(
            package='camera_cpp_nodes',
            executable='segmenter_node',
            name='segmenter_node',
            output='screen'
        ),        

        # 3. EXTRACTOR NODE (Puntos candidatos de carril/calzada)
        Node(
            package='camera_cpp_nodes',
            executable='extractor_node',
            name='extractor_node',
            parameters=[
                {'row_step': 2},
                {'min_width': 2},
                {'road_kernel': 5}
            ],
            output='screen'
        ),

        # 4. IPM NODE (Transformación a Metros / PointCloud2)
        Node(
            package='camera_cpp_nodes',
            executable='ipm_node',
            name='ipm_node',
            parameters=[
                {'min_distance': 1.5},
                {'max_distance': 7.0}
            ],
            output='screen'
        ),   

        # 5. ESP32 CONTROL NODE (Comunicación con microcontrolador)
        Node(
            package='camera_cpp_nodes',
            executable='esp32_control',
            name='esp32_control',
            output='screen'
        ),      

        

        
    ])

"""
        
        
"""