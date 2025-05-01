import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    keyboard_node = Node(
        package = 'teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard'
    )

    return LaunchDescription([
        keyboard_node,
    ])

if __name__ == '__main__':
    generate_launch_description()