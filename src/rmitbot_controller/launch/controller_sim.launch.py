import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node

# ros2 launch rmitbot_controller controller_sim.launch.py
# This launch file is for SIMULATION ONLY.
# In simulation, Gazebo's gz_ros2_control plugin provides the controller_manager.
# We only need to:
#   1. Publish the robot_description (robot_state_publisher)
#   2. Spawn the controllers (joint_state_broadcaster, mecanum_drive_controller)

def generate_launch_description():
    
    # Path to the controller config file
    pkg_path_controller = get_package_share_directory("rmitbot_controller")
    config_controller = os.path.join(pkg_path_controller, 'config', 'rmitbot_controller.yaml')
    
    # Path to the package
    pkg_path_description = get_package_share_directory("rmitbot_description")
    urdf_path = os.path.join(pkg_path_description, 'urdf', 'rmitbot_sim.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', '\'', urdf_path, '\'']), value_type=str)
    
    # Publish the robot static TF from the urdf
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": True, 
                     "robot_description": robot_description}]
    )
    
    # joint_state_broadcaster (jsb): dynamic TF of the motor joints 
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    # controller: IK from Cartesian speed to motor speed command
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'mecanum_drive_controller','--param-file', config_controller,
            '--controller-ros-args', '-r mecanum_drive_controller/tf_odometry:=tf',
            '--controller-ros-args', '-r mecanum_drive_controller/reference:=cmd_vel',
            '--controller-ros-args', '-r mecanum_drive_controller/odometry:=odom',
            '--controller-ros-args', '-r mecanum_drive_controller/controller_state:=controller_state',
        ],
    )
    
    # controller must be spawned after the jsb
    controller_spawner_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher, 
        joint_state_broadcaster_spawner,
        controller_spawner_after_jsb,
    ])
