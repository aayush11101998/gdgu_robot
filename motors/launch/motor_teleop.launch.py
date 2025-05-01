import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

# --- CORRECTED IMPORTS ---
# General launch substitutions
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution # Keep this if needed elsewhere, though maybe not strictly here anymore
)
# ROS-specific substitutions
from launch_ros.substitutions import FindPackageShare # <-- CORRECT LOCATION
# --- END CORRECTED IMPORTS ---

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='hardware',
        description='Available modes: [hardware]'
    )

    # Read URDF file
    package_dir = get_package_share_directory('motors')
    urdf_path = os.path.join(package_dir, 'resource', 'my_robo_hardware.urdf')
    robot_description_content = ""
    try:
        with open(urdf_path, 'r') as urdf_file:
            robot_description_content = urdf_file.read()
    except FileNotFoundError:
        print(f"Error: URDF file not found at {urdf_path}")
    robot_description = {'robot_description': robot_description_content}

    # --- PATH CONSTRUCTION (Using correct FindPackageShare import) ---
    mode_launch_file_components = [
        FindPackageShare('motors'), # Now correctly imported
        '/launch/',
        LaunchConfiguration('mode'),
        '.launch.py'
    ]

    # --- END PATH CONSTRUCTION ---


    # Launch nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Include mode-specific launch file
    teleop_mode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mode_launch_file_components),
    )

 

    # Return launch description
    return LaunchDescription([
        mode_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        teleop_mode_launch,
    ])