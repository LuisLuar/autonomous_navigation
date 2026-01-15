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

        # Segmenter YOLOP node
        Node(
            package='perception_stack',
            executable='segmenter_yolop',
            output='screen',
        ),

        # Visualizer segmentation node
        Node(
            package='perception_stack',
            executable='visualizer_segmentation',
            output='screen',
        ),

        # Detector YOLO V11  node
        Node(
            package='perception_stack',
            executable='detector_yolo',
            output='screen',
        ),

        # Fusion node object detection
        Node(
            package='perception_stack',
            executable='object_fusion',
            output='screen',
        ),

        # Visualizer detector node
        Node(
            package='perception_stack',
            executable='visualizer_detector',
            output='screen',
        ),

        # Speed limiter nodes
        Node(
            package='perception_stack',
            executable='vision_speed_limiter',
            output='screen',
        ),

        # IPM node + lane detection
        Node(
            package='perception_stack',
            executable='ipm_lane_detection',
            output='screen',
        ),

        # Lane error robust  node
        Node(
            package='perception_stack',
            executable='lane_error',
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


    ])


"""        
        # LANE CENTER TRACKING
        Node(
            package='perception_stack',
            executable='tracker_center_ekf',
            output='screen',
        ),        

        # lane modeling
        Node(
            package='perception_stack',
            executable='lane_modeling',
            output='screen',
        ),


        # LANE MODEL TRACKING
        Node(
            package='perception_stack',
            executable='tracker_model_ekf',
            output='screen',
        ),
    
"""