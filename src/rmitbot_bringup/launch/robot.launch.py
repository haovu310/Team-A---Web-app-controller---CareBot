import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rmitbot_controller = get_package_share_directory('rmitbot_controller')
    pkg_rmitbot_bringup = get_package_share_directory('rmitbot_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Controller (Base + Motors + IMU)
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rmitbot_controller, 'launch', 'controller.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Lidar (RPLIDAR A1)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(pkg_rmitbot_bringup, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'),
        
        controller_launch,
        lidar_launch
    ])
