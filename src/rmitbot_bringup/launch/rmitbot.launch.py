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
    
    # Declare launch argument for enabling vision
    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable camera and AprilTag vision system'
    )
    
    # Launch rviz
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description,"launch","display.launch.py"),
    )
    
    
    # Launch the controller manager
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller,"launch","controller.launch.py"),
    )
        
    # Launch ekf node
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization,"launch","localization.launch.py"),
    )
    
    # Launch the rplidar hardware
    rplidar = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch", "rplidar.launch.py"),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )
    
    # Launch the mapping node
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping,"launch","mapping.launch.py"),
    )
    
    # Launch the twistmux instead of keyboard node only
    twistmux = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation,"launch","twistmux.launch.py"),
    )
    
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation,"launch","nav.launch.py"),
    )
    
    # Launch vision system (camera + AprilTag detection)
    vision = IncludeLaunchDescription(
        os.path.join(pkg_path_vision,"launch","vision.launch.py"),
        condition=IfCondition(LaunchConfiguration('enable_vision'))
    )
    
    # Launch the navigation 10s after slamtoolbox, to make sure that a map is available
    navigation_delayed = TimerAction(
        period = 5., 
        actions=[navigation]
    )
    
    # PC: display, twistmux, navigation_delayed, vision
    # RPI: controller, localization, rplidar, mapping, vision
    return LaunchDescription([
        enable_vision_arg,
        display,
        # controller,
        twistmux,
        # localization,
        # rplidar, 
        # mapping,
        vision, 
        navigation_delayed, 
    ])