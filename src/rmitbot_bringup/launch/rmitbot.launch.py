import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

# Launch the file
# ros2 launch rmitbot_bringup rmitbot.launch.py

def generate_launch_description():
    
    # Path to the package 
    pkg_path_description =  get_package_share_directory("rmitbot_description")
    pkg_path_controller =   get_package_share_directory("rmitbot_controller")
    pkg_path_localization = get_package_share_directory("rmitbot_localization")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_vision = get_package_share_directory("rmitbot_vision")
    pkg_path_web = get_package_share_directory("rmitbot_web_controller")
    
    # Declare launch argument for enabling vision
    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable camera and AprilTag vision system'
    )
    
    # ==================================================================================================
    #   SECTION 1: ROBOT / RASPBERRY PI
    #   Run this section on the Robot. Comment out SECTION 2 when running on the Robot.
    # ==================================================================================================
    
    # 1.1: Robot Controller (Motors, ros2_control, robot_state_publisher)
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller,"launch","controller.launch.py"),
    )
    
    # 1.2: Lidar Sensor
    rplidar = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch", "rplidar.launch.py"),
        launch_arguments={"use_sim_time": "False"}.items()
    )
    
    # 1.3: Localization (EKF)
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization,"launch","localization.launch.py"),
    )
    
    # 1.4: Vision System (Camera + AprilTag)
    vision = IncludeLaunchDescription(
        os.path.join(pkg_path_vision,"launch","vision.launch.py"),
        condition=IfCondition(LaunchConfiguration('enable_vision'))
    )
    
    # 1.5: Twist Multiplexer (Safety Control & Priority)
    # MUST run on Robot for safety and proper control handling
    twistmux = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation,"launch","twistmux.launch.py"),
    )
    
    # ==================================================================================================
    #   SECTION 2: WORKSTATION / PC
    #   Run this section on the PC. Comment out SECTION 1 when running on the PC.
    # ==================================================================================================
    
    # 2.1: Web Controller (Rosbridge + Web Server)
    web_controller = IncludeLaunchDescription(
        os.path.join(pkg_path_web, "launch", "web.launch.py"),
    )
    
    # 2.2: Visualization (RViz)
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description,"launch","display.launch.py"),
    )
    
    # 2.3: Mapping (SLAM Toolbox)
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch","mapping.launch.py"),
    )
    
    # 2.4: Navigation (Nav2)
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation,"launch","nav.launch.py"),
    )
    
    # Delayed Start for Navigation (Wait for Map/SLAM)
    navigation_delayed = TimerAction(
        period = 5., 
        actions=[navigation]
    )

    # ==================================================================================================
    #   LAUNCH DESCRIPTION
    #   Uncomment the lines corresponding to your current setup (PC or Robot)
    # ==================================================================================================
    return LaunchDescription([
        enable_vision_arg,
        
        # --- ROBOT SECTION (Uncomment for Pi) ---
        controller,
        rplidar,
        localization,
        vision,
        twistmux,     

        # --- PC SECTION (Uncomment for Workstation) ---
        web_controller,
        # display,            # Run on PC/VNC if needed
        mapping,            # Enable for SLAM
        navigation_delayed, # Enable for Nav2
        
        # Note: 'vision' can also run on PC if testing with USB webcam
        # vision, 
    ])