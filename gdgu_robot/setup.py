from setuptools import find_packages, setup
from glob import glob

package_name = 'gdgu_robot'

data_files = []
data_files.append(('share/' + package_name + '/launch', ['launch/spawn.launch.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/gazebo_spawn.launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/rviz_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/tb3_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/localization_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/navigation_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/slam_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/bringup_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/nav_comm_robot_handler_launch.py']))

data_files.append(('share/' + package_name + '/resource', [
#     'resources/nav2_default_view.rviz',
#     'resources/joint_names_Final_assembly_for gazebo.yaml',
      'resource/gdgu_robot.sdf',
      'resource/robot.xacro',
      'resource/gazebo_control.xacro',
      'resource/robot_body.xacro',
      'resource/gdgu_robot.urdf.xacro',
      'resource/robot_inertia.xacro',
      'resource/ultrasonic.xacro'
])) 
data_files.append(('share/' + package_name + '/meshes', ['meshes/chassis.STL',
        'meshes/fixed_wheel.STL',
        'meshes/ULTRASONIC_INFRARED.STL']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/config', ['config/gz_bridge.yaml']))
#data_files.append(('share/' + package_name + '/world', ['world/urdf/Pipeline.sdf']))
#data_files.append(('share/' + package_name + '/world', ['world/urdf/robot.sdf']))
#data_files.append(('share/' + package_name + '/world', ['world/urdf/robot_new.sdf']))
#data_files.append(('share/' + package_name + '/world', ['world/urdf/swimming robot_assembly_SLDPRT.urdf']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/gdgu_robot.rviz']))
data_files.append(('share/' + package_name + '/world', ['world/empty.sdf']))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='aayush1110',
    maintainer_email='aayush.vats@gdgu.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    keywords=['ROS2', 'Gazebo ignition', 'gdgu_Robot', 'Simulation'],
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
        ],
    },
)
