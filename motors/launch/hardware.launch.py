import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    motor_controller_node = Node(
        package='motors',
        executable='motor_control',
        name='motor_control',
        output='screen'
    )
    
    motor_interface_node = Node(
        package='motors',
        executable='motor_driver_ros_interface',
        name='motor_driver_ros_interface',
        output='screen'
    )
    # Declare the launch arguments

    return LaunchDescription([
        motor_controller_node,
        motor_interface_node
])
  
  