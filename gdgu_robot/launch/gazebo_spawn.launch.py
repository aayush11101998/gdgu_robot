import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gdgu_robot'), 'launch', 'spawn.launch.py')
            ]),
            launch_arguments= {'use_sim_time':'true'}.items()
            )
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments= {'use_sim_time':'true'}.items()
            )
    
    spawn_robot = Node(package='ros_gz_sim', 
                       executable='create',
                       arguments=['-topic', 'robot_description',
                                  '-entity', 'gdgu_robot'], 
                       output= 'screen')
    
    return LaunchDescription([
            spawn_launch,
            gazebo_launch,
            spawn_robot,
    ])
    
    
