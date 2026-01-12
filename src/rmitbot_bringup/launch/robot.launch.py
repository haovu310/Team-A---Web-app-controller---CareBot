import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# =============================================================================
#   ROBOT LAUNCH FILE - Run this on the Raspberry Pi
# =============================================================================
# This launches ONLY hardware drivers. SLAM, Nav2, and the web interface
# run on the PC for better performance.
#
# Usage:
#   ros2 launch rmitbot_bringup robot.launch.py
#   ros2 launch rmitbot_bringup robot.launch.py serial_port_esp32:=/dev/ttyUSB0 serial_port_lidar:=/dev/ttyUSB1
#
# Make sure to set CycloneDDS environment:
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI=file:///path/to/cyclonedds_pi.xml
# =============================================================================

def generate_launch_description():
    
    # Package paths
    pkg_path_controller =   get_package_share_directory("rmitbot_controller")
    pkg_path_localization = get_package_share_directory("rmitbot_localization")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_vision = get_package_share_directory("rmitbot_vision")
    pkg_path_web = get_package_share_directory("rmitbot_web_controller")
    
    # === LAUNCH ARGUMENTS ===
    
    # Serial port for ESP32 motor controller
    serial_port_esp32_arg = DeclareLaunchArgument(
        'serial_port_esp32',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 motor controller'
    )
    serial_port_esp32 = LaunchConfiguration('serial_port_esp32')
    
    # Serial port for LiDAR
    serial_port_lidar_arg = DeclareLaunchArgument(
        'serial_port_lidar',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLidar'
    )
    serial_port_lidar = LaunchConfiguration('serial_port_lidar')
    
    # Enable vision system (camera + AprilTag)
    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='false',
        description='Enable camera and AprilTag vision system'
    )
    
    # === HARDWARE DRIVERS (Run on Pi) ===
    
    # 1. Robot Controller (Motors, ros2_control, robot_state_publisher)
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller, "launch", "controller.launch.py"),
        launch_arguments={'serial_port': serial_port_esp32}.items()
    )
    
    # 2. Lidar Sensor
    rplidar = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping, "launch", "rplidar.launch.py"),
        launch_arguments={'serial_port': serial_port_lidar}.items()
    )
    
    # 3. Localization (EKF) - Sensor fusion for odometry
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization, "launch", "localization.launch.py"),
    )
    
    # 4. Twist Multiplexer (Safety Control & Priority)
    twistmux = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "twistmux.launch.py"),
    )
    
    # 5. Vision System (Camera + AprilTag) - Optional
    vision = IncludeLaunchDescription(
        os.path.join(pkg_path_vision, "launch", "vision.launch.py"),
        condition=IfCondition(LaunchConfiguration('enable_vision'))
    )
    
    # 6. Camera Stream Server (for web UI) - Always on Pi
    # Note: Only camera_stream, NOT the web server or rosbridge
    # Rosbridge and web server run on PC for better performance
    from launch_ros.actions import Node
    camera_stream = Node(
        package='rmitbot_web_controller',
        executable='camera_stream',
        name='camera_stream',
        output='screen',
        parameters=[{
            'stream_port': 8001,
            'max_fps': 15,
            'camera_topic': 'camera/image_raw/compressed'
        }]
    )
    
    return LaunchDescription([
        # Arguments
        serial_port_esp32_arg,
        serial_port_lidar_arg,
        enable_vision_arg,
        
        # Hardware drivers
        controller,
        rplidar,
        localization,
        twistmux,
        vision,
        camera_stream,
    ])
