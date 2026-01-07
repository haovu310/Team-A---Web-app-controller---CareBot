from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ])
    
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
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
        web_video_server
    ])
