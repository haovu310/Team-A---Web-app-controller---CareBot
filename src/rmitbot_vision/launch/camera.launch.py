import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Pi Camera Module v3 using camera_ros package.
    This assumes the camera_ws is installed on the RPI at ~/camera_ws
    """
    
    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera image width (320, 640, 1280)'
    )
    
    height_arg = DeclareLaunchArgument(
        'height', 
        default_value='480',
        description='Camera image height (240, 480, 720)'
    )
    
    format_arg = DeclareLaunchArgument(
        'format',
        default_value='YUYV',
        description='Image format (YUYV recommended for Pi Camera)'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    # Camera node from camera_ros package
    # Note: This package must be sourced from ~/camera_ws on the RPI
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'format': LaunchConfiguration('format'),
            'framerate': LaunchConfiguration('framerate'),
        }],
        # Remap topics to standard camera namespace
        remappings=[
            ('image_raw', 'camera/image_raw'),
            ('image_raw/compressed', 'camera/image_raw/compressed'),
            ('camera_info', 'camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        width_arg,
        height_arg,
        format_arg,
        framerate_arg,
        camera_node,
    ])
