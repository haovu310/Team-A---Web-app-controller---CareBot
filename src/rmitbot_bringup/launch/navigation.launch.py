import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rmitbot_bringup = get_package_share_directory('rmitbot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file',
                                      default=os.path.join(pkg_rmitbot_bringup, 'config', 'nav2_params.yaml'))
    
    # Map is optional here. If provided, map_server loads it.
    # For SLAM navigation, we might run this without a map or with a blank map.
    map_dir = LaunchConfiguration('map',
                                  default=os.path.join(pkg_rmitbot_bringup, 'maps', 'my_map.yaml'))


    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_dir,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_rmitbot_bringup, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_rmitbot_bringup, 'maps', 'my_map.yaml'),
            description='Full path to map yaml file to load'),

        nav2_launch
    ])
