import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# =============================================================================
#   PC LAUNCH FILE - Run this on your laptop/workstation
# =============================================================================
# This launches compute-heavy nodes that benefit from PC's processing power:
# - SLAM Toolbox
# - Nav2 Navigation
# - RViz visualization
# - Rosbridge (for web UI)
# - Web Server
#
# The robot hardware (motors, lidar, camera) runs on the Pi via robot.launch.py
#
# Usage:
#   ros2 launch rmitbot_bringup pc.launch.py
#   ros2 launch rmitbot_bringup pc.launch.py enable_slam:=true enable_nav:=true
#
# Make sure to set CycloneDDS environment:
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI=file:///path/to/cyclonedds_pc.xml
#
# Web UI Access:
#   http://localhost:8000?rpi_host=<PI_IP_ADDRESS>
# =============================================================================

def generate_launch_description():
    
    # Package paths
    pkg_path_description = get_package_share_directory("rmitbot_description")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_web = get_package_share_directory("rmitbot_web_controller")
    
    # === LAUNCH ARGUMENTS ===
    
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM Toolbox for mapping'
    )
    
    enable_nav_arg = DeclareLaunchArgument(
        'enable_nav',
        default_value='true',
        description='Enable Nav2 navigation stack'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    # === COMPUTE NODES (Run on PC) ===
    
    # 1. RViz Visualization
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description, "launch", "display.launch.py"),
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # 2. SLAM Toolbox (Mapping)
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping, "launch", "mapping.launch.py"),
        condition=IfCondition(LaunchConfiguration('enable_slam'))
    )
    
    # 3. Nav2 Navigation (delayed to wait for map)
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "nav.launch.py"),
        condition=IfCondition(LaunchConfiguration('enable_nav'))
    )
    navigation_delayed = TimerAction(
        period=5.0,
        actions=[navigation]
    )
    
    # 4. Rosbridge WebSocket (for web UI communication)
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9091,
            'max_message_size': 10000000,
            'unregister_timeout': 10.0,
            'fragment_timeout': 600,
            'delay_between_messages': 0,
            'max_queue_size': 10,
            'use_compression': True,
            'authenticate': False,
            'retry_startup_delay': 5.0,
        }]
    )
    
    # 5. Web Server (serves HTML/CSS/JS)
    web_server = Node(
        package='rmitbot_web_controller',
        executable='web_server',
        name='web_server',
        output='screen',
        parameters=[{
            'port': 8000
        }]
    )
    
    return LaunchDescription([
        # Arguments
        enable_slam_arg,
        enable_nav_arg,
        enable_rviz_arg,
        
        # Compute nodes
        display,
        mapping,
        navigation_delayed,
        rosbridge,
        web_server,
    ])
