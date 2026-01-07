import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Path to the package 
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
    # In simulation, the controller_manager is provided by Gazebo's gz_ros2_control plugin.
    # We only spawn the controllers (not the ros2_control_node).
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
    
    # Launch Mapping (SLAM Toolbox)
    mapping = IncludeLaunchDescription(
        os.path.join(pkg_path_mapping, "launch", "mapping.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Launch Navigation (Nav2) - delayed to ensure SLAM has a map
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_path_navigation, "launch", "nav.launch.py"),
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
        gazebo,
        controller_delayed,
        web_controller,
        twistmux,
        localization,
        mapping,
        navigation_delayed,
        display
    ])
