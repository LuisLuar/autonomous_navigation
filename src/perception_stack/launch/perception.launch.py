from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # Capture frame node
        Node(
            package='perception_stack',
            executable='capture_frame',
            output='screen',
        ),

        # =========== SEGMENTER YOLOP ===========
        Node(
            package='perception_stack',
            executable='segmenter_yolop',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='segmenter_yolop_left',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='segmenter_yolop_right',
            output='screen',
        ),

        # =========== CORRECCION COLOR ===========
        Node(
            package='perception_stack',
            executable='correct_color_left',
            output='screen',
        ),    

        Node(
            package='perception_stack',
            executable='correct_color_right',
            output='screen',
        ),        

        # =========== CANNY extraccion de pixeles candidatos ===========
        Node(
            package='perception_stack',
            executable='canny_left',
            output='screen',
        ),  

        Node(
            package='perception_stack',
            executable='canny_right',
            output='screen',
        ), 

        Node(
            package='perception_stack',
            executable='canny',
            output='screen',
        ),  

        # =========== PIXEL A METRO ===========
        Node(
            package='perception_stack',
            executable='pixel_to_meter_front',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='pixel_to_meter_left',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='pixel_to_meter_right',
            output='screen',
        ),

        # =========== FUSION DE LAS 3 CAMARAS ===========
        Node(
            package='perception_stack',
            executable='lane_pointcloud_merger',
            output='screen',
        ),

        # =========== DETECCION POSIBLES LINEAS ===========
        Node(
            package='perception_stack',
            executable='lane_detector',
            output='screen',
        ), 

        # =========== MEMORIA LOCAL Y FILTRADO TOPOLOGICO ===========
        Node(
            package='perception_stack',
            executable='topology',
            output='screen',
        ), 

        # =========== MEMORIA GLOBAL ===========
        Node(
            package='perception_stack',
            executable='lane_global_memory',
            output='screen',
        ),

        # =========== FILTRADO ===========
        Node(
            package='perception_stack',
            executable='filtered',
            output='screen',
        ),   

        # =========== GENERAR TARGET ============
        Node(
            package='perception_stack',
            executable='generate_target',
            output='screen',
        ),


        # =========== YOLO V11 ===========
        Node(
            package='perception_stack',
            executable='detector_yolo',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='detector_senaletica',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='object_fusion',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='object_to_odom',
            output='screen',
        ), 

        # ============ Speed limiter nodes =============
        Node(
            package='perception_stack',
            executable='vision_speed_limiter',
            output='screen',
        ),

        # =========== VISUALIZADOR ===========
        Node(
            package='perception_stack',
            executable='visualizer_camera_front',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='visualizer_camera_left',
            output='screen',
        ),    

         Node(
            package='perception_stack',
            executable='visualizer_camera_right',
            output='screen',
        ), 
       
    ])



"""  



        # Detector YOLO V11 LEFT
        Node(
            package='perception_stack',
            executable='detector_yolo_left',
            output='screen',
        ),

        # Detector YOLO V11 RIGHT
        Node(
            package='perception_stack',
            executable='detector_yolo_right',
            output='screen',
        ),

        # Fusion node object detection LEFT
        Node(
            package='perception_stack',
            executable='object_fusion_left',
            output='screen',
        ),

        # Fusion node object detection RIGHT
        Node(
            package='perception_stack',
            executable='object_fusion_right',
            output='screen',
        ),

        # Lane error robust  node
        Node(
            package='perception_stack',
            executable='lane_error',
            output='screen',
        ),

        # IPM node + lane detection
        Node(
            package='perception_stack',
            executable='ipm_lane_detection',
            output='screen',
        ),

        # IPM SOLO PARA DEBUG
        Node(
            package='perception_stack',
            executable='ipm',
            output='screen',
        ),

        

        

        # BEV CENTER DEBUG
        Node(
            package='perception_stack',
            executable='bev_center_debug',
            output='screen',
        ),


    
"""