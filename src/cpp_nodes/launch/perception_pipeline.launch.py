import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Rutas a archivos de parámetros (ajusta si es necesario)
     # Rutas a archivos de parámetros (ajusta si es necesario)
    calibration_file = "/home/raynel/autonomous_navigation/src/perception_stack/params/calibration_front.json"

    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )

    # Rutas a los archivos YAML específicos
    speed_yaml = os.path.join(global_params_path, 'speed_table.yaml')

    # Definimos la ruta del engine de TwinLite
    twinlite_engine_path = os.path.join(global_params_path, 'twinlite_512x288_lines_chunks.engine')
    yolopv2_engine_path = os.path.join(global_params_path, 'yolopv2_lane_288_OMEN_TRT10.engine')
    yolov11_engine_path = os.path.join(global_params_path, 'yolopv2_lane_288_OMEN_TRT10.engine')

    return LaunchDescription([
        # 1. CAMERA NODE (Captura y Redimensión)
        Node(
            package='cpp_nodes',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device': 0}],
            output='screen'
        ),

        # 2. SEGMENTER NODE (YOLOPv2 Inferencia)
        Node(
            package='cpp_nodes',
            executable='segmenter_node',
            name='segmenter_node',
            output='screen'
        ),        

        # 4. IPM NODE (Transformación a Metros / PointCloud2)
        Node(
            package='cpp_nodes',
            executable='ipm_node',
            name='ipm_node',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 30.0}
            ],
            output='screen'
        ), 

        # 5. ESP32 CONTROL NODE (Comunicación con microcontrolador)
        Node(
            package='cpp_nodes',
            executable='esp32_control',
            name='esp32_control',
            output='screen'
        ), 

        

        
    ])

"""
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

          

         
        
"""