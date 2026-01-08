from launch import LaunchDescription
from launch_ros.actions import Node
import sys

def generate_launch_description():
    """
    DEPRECATED: This launch file is kept for backward compatibility.
    
    For distributed setup (RECOMMENDED):
    - On PC: Use web_pc.launch.py (only web server)
    - On RPi: Use web_rpi.launch.py (rosbridge + camera stream)
    
    This file starts ALL services, which is only suitable if running
    everything on a single machine (typically the RPi for standalone operation).
    """
    print("\n" + "="*70)
    print("WARNING: web.launch.py is DEPRECATED")
    print("="*70)
    print("For distributed setup, use:")
    print("  - PC:  ros2 launch rmitbot_web_controller web_pc.launch.py")
    print("  - RPi: ros2 launch rmitbot_web_controller web_rpi.launch.py")
    print("="*70 + "\n")
    sys.stdout.flush()
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
        # Start camera stream server
        Node(
            package='rmitbot_web_controller',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
            parameters=[{'stream_port': 8001}]
        )
    ])
