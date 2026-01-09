from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic',
            default_value='camera/image_raw/compressed',
            description='Topic for camera stream'
        ),
        DeclareLaunchArgument(
            'use_compressed',
            default_value='True',
            description='Whether to use compressed image transport'
        ),

        # Start rosbridge server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}] 
        ),
        # Start python web server
        Node(
            package='rmitbot_web_controller',
            executable='web_server',
            name='web_server',
            output='screen',
            parameters=[{'port': 8000}]
        ),
        # Start camera stream server
        Node(
            package='rmitbot_web_controller',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
            parameters=[{
                'stream_port': 8001,
                'camera_topic': LaunchConfiguration('camera_topic'),
                'use_compressed': LaunchConfiguration('use_compressed')
            }]
        )
    ])
