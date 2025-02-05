import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    #use_ros2_control = LaunchConfiguration('use_ros2_control')
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('gdgu_robot'))
    xacro_file = os.path.join(pkg_path,'resource','gdgu_robot.sdf')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    with open(xacro_file, 'r') as f:
        robot_description_config = f.read()
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        node_robot_state_publisher,
    ])