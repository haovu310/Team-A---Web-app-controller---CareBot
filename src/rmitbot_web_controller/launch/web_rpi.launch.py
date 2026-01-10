from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Raspberry Pi - OPTIMIZED FOR PERFORMANCE
    Starts rosbridge and camera stream services with reduced CPU usage.

    This should run on the robot (Raspberry Pi) alongside the robot controller,
    sensors, and actuators.

    OPTIMIZATIONS:
    - Reduced rosbridge message queue size
    - Optimized fragment and delay thresholds
    - Limited camera FPS to 15 for lower CPU usage
    - Enabled unregister timeout to clean up stale connections
    - Compression enabled for bandwidth efficiency

    Services started:
    - rosbridge_websocket (port 9090) - WebSocket bridge for web UI
    - camera_stream (port 8001) - MJPEG camera stream server

    Usage on Raspberry Pi:
        ros2 launch rmitbot_web_controller web_rpi.launch.py
    """
    return LaunchDescription([
        # Start rosbridge server with optimized parameters
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                # Performance optimizations
                'max_message_size': 10000000,  # 10MB limit
                'unregister_timeout': 10.0,     # Clean up stale subscriptions
                'fragment_timeout': 600,         # Fragment assembly timeout
                'delay_between_messages': 0,     # No artificial delay
                'max_queue_size': 10,           # Limit queue to prevent memory buildup
                # Compression and throttling
                'use_compression': True,         # Enable compression for large messages
                'authenticate': False,           # Disable auth for local network
                # Connection management
                'retry_startup_delay': 5.0,     # Retry delay on startup
            }]
        ),
        # Start camera stream server with FPS limit
        Node(
            package='rmitbot_web_controller',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
            parameters=[{
                'stream_port': 8001,
                'max_fps': 15,  # Limit to 15 FPS for Raspberry Pi
                'camera_topic': 'camera/image_raw/compressed'
            }]
        )
    ])

