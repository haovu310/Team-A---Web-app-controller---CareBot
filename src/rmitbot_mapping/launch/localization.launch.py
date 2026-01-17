import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# Launch SLAM Toolbox in LOCALIZATION mode (for using pre-saved maps)
# ros2 launch rmitbot_mapping localization.launch.py use_sim_time:=true

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    slam_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("slam_toolbox"),"launch","localization_launch.py"),
        launch_arguments={
            'params_file': os.path.join(get_package_share_directory("rmitbot_mapping"), "config", "slam_localization.yaml"),             
            'use_sim_time': use_sim_time, 
            }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        slam_launch,
    ])
