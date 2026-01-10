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
    # This path works dynamically for any user (stored in home directory)
    maps_dir = Path.home() / 'Team-A---Web-app-controller---CareBot' / 'maps'
    maps_dir.mkdir(parents=True, exist_ok=True)
    
    # Define the map file name parameter (full path + filename prefix)
    map_file_name = str(maps_dir / 'default')

    # Reference paths (as requested):
    # Aesitof's path: /home/aesitof/Team-A---Web-app-controller---CareBot/maps/default
    # Hao's path: /home/hao/Documents/App Control/team_A_web_app/src/rmitbot_mapping/maps/default
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    # Launch the SLAM node directly to allow parameter overrides
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            # Load default params from YAML
            os.path.join(get_package_share_directory("rmitbot_mapping"), "config", "slam.yaml"),
            # Override with dynamic values
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file_name
            }
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        slam_node,
    ])
    