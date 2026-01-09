import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from pathlib import Path

# ros2 launch rmitbot_mapping slam.launch.py use_sim_time:=true
# ros2 launch rmitbot_mapping slam.launch.py use_sim_time:=false
# Command line
# ros2 launch slam_toolbox online_async_launch.py params_file:=./scr/rmitbot_mapping/config/slam.yaml use_sim_time:=true

def generate_launch_description():
    
    # Create maps directory if it doesn't exist
    maps_dir = Path.home() / 'Team-A---Web-app-controller---CareBot' / 'maps'
    maps_dir.mkdir(parents=True, exist_ok=True)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    slam_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("slam_toolbox"),"launch","online_async_launch.py"),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory("rmitbot_mapping"), "config", "slam.yaml"),             
            'use_sim_time': use_sim_time, 
            }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        slam_launch,
    ])
    