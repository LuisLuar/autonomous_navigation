import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Rutas a archivos de parámetros (ajusta si es necesario)
    calibration_file = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_front.json"

    return LaunchDescription([
        # 1. CAMERA NODE (Captura y Redimensión)
        Node(
            package='cpp_nodes',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device': 2}],
            output='screen'
        ),

        # 2. SEGMENTER NODE (YOLOPv2 Inferencia)
        Node(
            package='cpp_nodes',
            executable='segmenter_node',
            name='segmenter_node',
            output='screen'
        ),        

        # 3. EXTRACTOR NODE (Puntos candidatos de carril/calzada)
        Node(
            package='cpp_nodes',
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
            package='cpp_nodes',
            executable='segmenter_to_meter',
            name='ipm_node',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 30.0}
            ],
            output='screen'
        ),   

        # 5. DETECT NODE (YOLOv11n Inferencia)
        Node(
            package='cpp_nodes',
            executable='detect_node',
            name='detect_node',
            output='screen'
        ),

        # 6. IPM NODE (Transformación a Metros / PointCloud2)
        Node(
            package='cpp_nodes',
            executable='object_to_meter',
            name='ipm_node',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 10.0}
            ],
            output='screen'
        ),  

        # 7. ESP32 CONTROL NODE (Comunicación con microcontrolador)
        Node(
            package='cpp_nodes',
            executable='esp32_control',
            name='esp32_control',
            output='screen'
        ),      

        
    ])

"""
        
        
"""