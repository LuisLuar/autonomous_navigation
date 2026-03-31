from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # ========= Capture frame node ===================
        Node(
            package='perception_stack',
            executable='capture_frame',
            output='screen',
        ),
        
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

        # ============ VISUALIZADORES =============
        Node(
            package='perception_stack',
            executable='visualizer_camera_front',
            output='screen',
        ),

        Node(
            package='perception_stack',
            executable='inverse_IPM',
            output='screen',
        ),

        
    ])



"""          

        Node(
            package='perception_stack',
            executable='inverse_IPM_CL',
            output='screen',
        ),


        # =========== YOLO V11 ===========
        Node(
            package='perception_stack',
            executable='detector_yolo',
            output='screen',
        ),

    
"""