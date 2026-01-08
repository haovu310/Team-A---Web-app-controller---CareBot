from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for PC workstation.
    Starts only the web server to serve the control interface.
    
    The web interface will connect to:
    - rosbridge_websocket on the RPi (port 9090)
    - camera_stream on the RPi (port 8001)
    
    Usage:
        ros2 launch rmitbot_web_controller web_pc.launch.py
        
    Then open browser to:
        http://localhost:8000
        
    If RPi IP is not the default, use URL parameter:
        http://localhost:8000?rpi_host=<RPi_IP>
    """
    return LaunchDescription([
        # Start python web server on PC
        Node(
            package='rmitbot_web_controller',
            executable='web_server',
            name='web_server',
            output='screen',
            parameters=[{'port': 8000}]
        )
    ])
