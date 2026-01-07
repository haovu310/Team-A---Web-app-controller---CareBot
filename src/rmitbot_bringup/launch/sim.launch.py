import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path_bringup = get_package_share_directory("rmitbot_bringup")
    pkg_path_description = get_package_share_directory("rmitbot_description")
    pkg_path_mapping = get_package_share_directory("rmitbot_mapping")
    # pkg_path_navigation = get_package_share_directory("rmitbot_navigation")
    pkg_path_web = get_package_share_directory("rmitbot_web_controller")
    pkg_path_vision = get_package_share_directory("rmitbot_vision")
    
    # 1. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_description, "launch", "gazebo.launch.py")
        )
    )

    # 2. Controllers (Spawn them for simulation)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    mecanum_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
        output="screen",
    )
    
    # 3. Web Controller
    web_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_web, "launch", "web.launch.py")
        )
    )
    
    # 4. SLAM / Mapping
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_mapping, "launch", "mapping.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    # 5. Navigation (included via rmitbot.launch or directly)
    # Using direct include here for clarity and sim_time enforcement
    # navigation = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_path_navigation, "launch", "nav.launch.py")
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items()
    # )
    
    # Reuse rmitbot.launch.py logic partially? 
    # Actually, simpler to just run navigation directly:
    rmitbot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_bringup, "launch", "rmitbot.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            # We don't want to launch hardware drivers in rmitbot.launch.py
            # But rmitbot.launch.py structure is:
            # - controller (hardware) -> SKIP
            # - localization (ekf) -> MAYBE? Gazebo usually provides odom
            # - nav -> YES
            # - mapping -> YES
        }.items()
    )
    
    # BETTER STRATEGY: 
    # Since rmitbot.launch.py is messy with hardware vs PC sections, let's explicitly run what we need here.
    
    # For sim, we need:
    # - Gazebo (provides clock, link states)
    # - Robot State Publisher (in gazebo.launch.py usually, or needs separate)
    # - Joint State Broadcaster (Spawmed above)
    # - Mecanum Controller (Spawned above)
    # - Web Server
    # - SLAM (Mapping)
    # - Navigation

    # Let's add robot_state_publisher here if gazebo.launch.py doesn't do it cleanly with sim time
    # Checking gazebo.launch.py: It spawns robot but doesn't seem to start RSP? 
    # Wait, gazebo.launch.py just includes gz_sim and spawns entity.
    # We need RSP!

    # Use Command to generate robot_description cleanly
    robot_description = Command(['xacro ', os.path.join(pkg_path_description, "urdf", "rmitbot.urdf.xacro"), ' use_sim:=true'])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(robot_description, value_type=str)
        }]
    )

    return LaunchDescription([
        gazebo,
        rsp,
        joint_state_broadcaster,
        mecanum_drive_controller,
        mapping, # This starts SLAM
        # navigation, # Add navigation later or via CLI if needed. Or uncomment above.
        web_controller
    ])
