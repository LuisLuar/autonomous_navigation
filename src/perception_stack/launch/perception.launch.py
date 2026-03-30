from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # ============ VISUALIZADORES =============
        Node(
            package='perception_stack',
            executable='visualizer_camera_front',
            output='screen',
        ),
        
        

        
        

    ])



"""       
        # ===============PAPER =============
        Node(
            package='perception_stack',
            executable='lane_model_estimator',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='lane_model_tracking',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='inverse_IPM',
            output='screen',
        ),

        # ========= Capture frame node ===================
        Node(
            package='perception_stack',
            executable='capture_frame',
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

        # ============ Speed limiter nodes =============
        Node(
            package='perception_stack',
            executable='vision_speed_limiter',
            output='screen',
        ),


        # ===============================
        Node(
            package='perception_stack',
            executable='object_to_odom',
            output='screen',
        ), 
    
"""