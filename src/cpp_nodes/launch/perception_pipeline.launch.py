import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )

    # Rutas a los archivos YAML específicos
    speed_yaml = os.path.join(global_params_path, 'speed_table.yaml')

    # Definimos la ruta del engine de TwinLite
    twinlite_engine_path = os.path.join(global_params_path, 'twinlite_FAST_32bit_RTX4060.engine')
    yolov11_engine_path = os.path.join(global_params_path, 'detect_yolo_512x288_RTX4060.engine')
    camera_calibration_path = os.path.join(global_params_path, 'camera_calibration.json')
    yolopv2_engine_path = os.path.join(global_params_path, 'yolopv2_lane_288_OMEN_TRT10.engine')
    class_thresholds_yaml = os.path.join(global_params_path, 'class_thresholds.yaml')  


    return LaunchDescription([
        # 1. CAMERA NODE (Captura y Redimensión)
        Node(
            package='cpp_nodes',
            executable='camera_node',
            name='camera_node',
            parameters=[{'device': 2}],
            output='screen'
        ),

                 # 2. SEGMENTER NODE modificado
        Node(
            package='cpp_nodes',
            executable='segmenter_twin_node',
            name='segmenter_twin_node',
            parameters=[{
                'engine_path': twinlite_engine_path
            }],
            output='screen'
        ),       

        # 4. IPM NODE segmentation (Transformación a Metros / PointCloud2)
        Node(
            package='cpp_nodes',
            executable='segmenter_to_meter',
            name='ipm_segmentation',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 30.0},
                {'config_path': camera_calibration_path}
            ],
            output='screen'
        ),   

        # 5. DETECT NODE (YOLOv11n Inferencia)
        Node(
            package='cpp_nodes',
            executable='detect_node',
            name='detect_node',
            parameters=[{
                'engine_path': yolov11_engine_path},
                class_thresholds_yaml
                ],
            output='screen'
        ),


        # 6. IPM NODE object (Transformación a Metros / PointCloud2)
        Node(
            package='cpp_nodes',
            executable='object_to_meter',
            name='ipm_object',
            parameters=[
                {'min_distance': 0.0},
                {'max_distance': 10.0},
                {'config_path': camera_calibration_path},
                speed_yaml    # La ruta del archivo va sola en la lista
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