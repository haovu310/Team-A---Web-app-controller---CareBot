import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

# Launch the simulation
# ros2 launch rmitbot_bringup simulation.launch.py

def generate_launch_description():
    
    # Path to the packages
    pkg_path_description =  get_package_share_directory("rmitbot_description")
    pkg_path_controller =   get_package_share_directory("rmitbot_controller")
    pkg_path_localization = get_package_share_directory("rmitbot_localization")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_web = get_package_share_directory("rmitbot_web_controller")
    
    # ==================================================================================================
    #   GAZEBO SIMULATION LAUNCH
    # ==================================================================================================
    
    # 1. Gazebo Simulation Environment
    gazebo = IncludeLaunchDescription(
        os.path.join(pkg_path_description, "launch", "gazebo.launch.py"),
    )
    
    # 2. Robot Controller (Simulated via Gazebo ros2_control)
    # This uses the Gazebo plugin instead of hardware interface
    controller = IncludeLaunchDescription(
        os.path.join(pkg_path_controller, "launch", "controller.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # 3. Localization (EKF with simulated sensors)
    localization = IncludeLaunchDescription(
        os.path.join(pkg_path_localization, "launch", "localization.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # 4. Visualization (RViz)
    display = IncludeLaunchDescription(
        os.path.join(pkg_path_description, "launch", "display.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # 5. Mapping (SLAM Toolbox)
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping, "launch", "mapping.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # 6. Twist Multiplexer (Control Priority)
    twistmux = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "twistmux.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # 7. Navigation (Nav2)
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "nav.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items()
    )
    
    # Delayed Start for Navigation (Wait for Map/SLAM)
    navigation_delayed = TimerAction(
        period = 5.0, 
        actions=[navigation]
    )
    
    # 8. Web Controller (Rosbridge + Web Server)
    web_controller = IncludeLaunchDescription(
        os.path.join(pkg_path_web, "launch", "web.launch.py"),
    )

    # ==================================================================================================
    #   LAUNCH DESCRIPTION
    # ==================================================================================================
    return LaunchDescription([
        # Core simulation
        gazebo,
        controller,
        localization,
        
        # Visualization and navigation
        display,
        mapping,
        twistmux,
        navigation_delayed,
        
        # Web interface
        web_controller,
    ])
