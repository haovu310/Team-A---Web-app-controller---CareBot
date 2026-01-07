import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

# Complete simulation launch with optional mapping or navigation modes
# 
# MAPPING MODE (default):
#   ros2 launch rmitbot_bringup simulation_complete.launch.py mode:=mapping
#
# NAVIGATION MODE (load existing map):
#   ros2 launch rmitbot_bringup simulation_complete.launch.py mode:=navigation
#

def generate_launch_description():
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Mode: "mapping" (SLAM) or "navigation" (use saved map)'
    )
    
    mode = LaunchConfiguration('mode')
    
    # Path to the packages 
    pkg_path_description =  get_package_share_directory("rmitbot_description")
    pkg_path_controller =   get_package_share_directory("rmitbot_controller")
    pkg_path_localization = get_package_share_directory("rmitbot_localization")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_web = get_package_share_directory("rmitbot_web_controller")

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        os.path.join(pkg_path_description, "launch", "gazebo.launch.py"),
    )
    
    # Launch Controllers (Joint State Broadcaster + Mecanum Drive)
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller,"launch","controller_sim.launch.py"),
    )
    
    controller_delayed = TimerAction(
        period = 3.0, 
        actions=[controller]
    )

    # Launch Web Controller (Rosbridge + Web Server)
    web_controller = IncludeLaunchDescription(
        os.path.join(pkg_path_web, "launch", "web.launch.py"),
    )
    
    # Launch Twist Mux (routes cmd_vel_keyboard -> cmd_vel)
    twistmux = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "twistmux.launch.py"),
    )
    
    # Launch Localization (EKF)
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization, "launch", "localization.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Launch Mapping (SLAM Toolbox) - Only in mapping mode
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping, "launch", "mapping.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=UnlessCondition(LaunchConfiguration('mode'))  # Will need proper condition
    )
    
    # Launch Localization with saved map - Only in navigation mode
    localization_with_map = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping, "launch", "localization.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('mode'))  # Will need proper condition
    )
    
    # Launch Navigation (Nav2) - delayed to ensure map is ready
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "nav.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    navigation_delayed = TimerAction(
        period = 10.0,
        actions=[navigation]
    )

    # Launch RViz
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description, "launch", "display.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        mode_arg,
        gazebo,
        controller_delayed,
        web_controller,
        twistmux,
        localization,
        mapping,  # Always launch mapping for now (can save maps via web interface)
        navigation_delayed,
        display
    ])
