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

    bridge_params = os.path.join(get_package_share_directory('gdgu_robot'),'config','gz_bridge.yaml')
       
    default_world = os.path.join(
        get_package_share_directory('gdgu_robot'),'world','empty.sdf')    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument('world',
        default_value=default_world,
        description='World to load'
        )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gdgu_robot'), 'launch', 'spawn.launch.py')
            ]),
            launch_arguments= {'use_sim_time':'true'}.items()
            )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ]))

    spawn_robot = Node(package='ros_gz_sim', 
                       executable='create',
                       arguments=['-topic', 'robot_description',
                                  '-name', 'gdgu_robot',
                                  '-z','0.1'], 
                       output= 'screen')
    
    ros_ign_bridge = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args',
                                     '-p',
                                     f'config_file:={bridge_params}',],
                          output='screen'  
    )
    
    return LaunchDescription([
            world_arg,
            spawn_launch,
            gazebo_launch,
            spawn_robot,
            ros_ign_bridge            
    ])
    
    
