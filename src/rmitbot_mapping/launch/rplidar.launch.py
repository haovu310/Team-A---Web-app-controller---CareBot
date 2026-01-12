import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch rmitbot_mapping rplidar.launch.py
# ros2 launch rmitbot_mapping rplidar.launch.py serial_port:=/dev/ttyUSB1

def generate_launch_description():
    
    # Declare serial_port argument for LiDAR
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLidar (e.g., /dev/ttyUSB1 or /dev/rplidar)'
    )
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        serial_port_arg,
        
        Node(
            package=    'rplidar_ros',
            executable= 'rplidar_composition',
            output=     'screen',
            parameters=[{
                'serial_port': serial_port,
                'frame_id': 'laser_link',
                'angle_compensate': True,
                'scan_mode': 'Standard', 
                'use_sim_time': False,
            }]
        )
    ])