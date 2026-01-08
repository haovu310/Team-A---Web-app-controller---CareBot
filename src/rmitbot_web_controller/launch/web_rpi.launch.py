from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Raspberry Pi.
    Starts rosbridge and camera stream services.
    
    This should run on the robot (Raspberry Pi) alongside the robot controller,
    sensors, and actuators.
    
    Services started:
    - rosbridge_websocket (port 9090) - WebSocket bridge for web UI
    - camera_stream (port 8001) - MJPEG camera stream server
    
    Usage on Raspberry Pi:
        ros2 launch rmitbot_web_controller web_rpi.launch.py
    """
    return LaunchDescription([
        # Start rosbridge server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}] 
        ),
        # Start camera stream server
        Node(
            package='rmitbot_web_controller',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
            parameters=[{'stream_port': 8001}]
        )
    ])
