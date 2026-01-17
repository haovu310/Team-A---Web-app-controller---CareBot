import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, LogInfo 
from launch_ros.actions import Node 
from launch.substitutions import LaunchConfiguration 

import os 
from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription 
from launch.actions import DeclareLaunchArgument, LogInfo 
from launch_ros.actions import Node 
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():
    # Optional: Image rectification node (usually not needed for AprilTag detection)
    # Uncomment if you experience distortion issues
    # image_proc_node = Node(
    #     package="image_proc",
    #     executable="rectify_node",
    #     name="rectify_node",
    #     output="screen",
    #     remappings=[ 
    #         ('image', 'camera/image_raw'), 
    #         ('camera_info', 'camera/camera_info'), 
    #         ('image_rect', 'camera/image_rect'),
    #     ], 
    # )
    
    apriltag_params = os.path.join(
        get_package_share_directory("rmitbot_vision"), 
        "config", 
        "apriltag_params.yaml"
    ) 
    
    apriltag_node = Node( 
        package='apriltag_ros', 
        executable='apriltag_node', 
        name='apriltag_node', 
        output='screen', 
        parameters=[
            apriltag_params,
            # Remove use_sim_time for real hardware
        ], 
        remappings=[ 
            # Use raw image directly from camera
            ("image_rect", "camera/image_raw"),
            ("camera_info", "camera/camera_info"), 
        ], 
    ) 

    return LaunchDescription([
        # Uncomment if using image rectification
        # image_proc_node, 
        apriltag_node, 
    ])


    
