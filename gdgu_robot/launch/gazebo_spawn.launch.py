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

    world_arg = DeclareLaunchArgument('world',
        default_value=os.path.join(get_package_share_directory('gdgu_robot'),'world','obstacles.sdf'),
        description='World to load'
        )
    
    world_file = LaunchConfiguration('world')
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(get_package_share_directory('gdgu_robot'),'rviz','gdgu_robot.rviz'),
        description='Full path to the RViz configuration file'
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
            ]),
            launch_arguments={
            'gz_args': [f'-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_robot = Node(package='ros_gz_sim', 
                       executable='create',
                       arguments=['-name', 'gdgu_robot',
                                  '-topic', 'robot_description',
                                  '-name', 'gdgu_robot',
                                  '-z','0.25'], 
                       output= 'screen')
    
    ros_ign_bridge = Node(package='ros_gz_bridge',
                          executable='parameter_bridge',
                          arguments=['--ros-args',
                                     '-p',
                                     f'config_file:={bridge_params}',],
                          output='screen',                     
    )

    '''odom_pub = Node(
        package='tf2_ros',
        executable = '',
        name = 'odom',
        arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters= ['publish_frequency:=100']
    )'''

    '''rwheelpub = Node(
        package='tf2_ros',
        executable = '',
        name = 'right_wheel_pub',
        arguments = ['0', '-0.155', '0', '0', '0', '1.57', 'base_link', 'rfw_link']
    )'''

    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d',LaunchConfiguration('rviz_config')],
                output='screen'
                )
    
    return LaunchDescription([
            world_arg,
            rviz_config_arg,
            ros_ign_bridge, 
            spawn_robot,   
            spawn_launch, 
            rviz,     
            gazebo_launch,
            #lwheelpub,
            #rwheelpub
    ])
    
    
