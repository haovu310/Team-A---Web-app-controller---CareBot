import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Combined launch file for RMITBot vision system.
    Launches:
    1. Pi Camera driver (camera_ros)
    2. AprilTag detection (apriltag_ros)
    """
    
    pkg_path_vision = get_package_share_directory("rmitbot_vision")
    
    # Declare launch arguments for camera configuration
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='Camera resolution width (320, 640, 1280)'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='Camera resolution height (240, 480, 720)'
    )
    
    # Launch camera driver
    camera_launch = IncludeLaunchDescription(
        os.path.join(pkg_path_vision, "launch", "camera.launch.py"),
        launch_arguments={
            'width': LaunchConfiguration('camera_width'),
            'height': LaunchConfiguration('camera_height'),
        }.items()
    )
    
    # Launch AprilTag detection
    apriltag_launch = IncludeLaunchDescription(
        os.path.join(pkg_path_vision, "launch", "apriltag.launch.py"),
    )
    
    return LaunchDescription([
        camera_width_arg,
        camera_height_arg,
        camera_launch,
        apriltag_launch,
    ])
