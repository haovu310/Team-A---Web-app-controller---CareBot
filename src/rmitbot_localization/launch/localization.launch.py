from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    pkg_path_localization = get_package_share_directory("rmitbot_localization")
    config_localization =   os.path.join(pkg_path_localization, 'config', 'ekf.yaml')

    robot_localization = Node(
        package=    "robot_localization",
        executable= "ekf_node",
        name=       "ekf_filter_node",
        output=     "screen",
        parameters=[config_localization, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom_ekf')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_localization,
    ])