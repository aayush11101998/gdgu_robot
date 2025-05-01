from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'motors'

data_files=[]

data_files.append((os.path.join('share', package_name, 'launch'), glob('launch/*.py')))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robo_hardware.urdf']))
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))



setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab110',
    maintainer_email='lab110@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],
        'console_scripts': [
        'motor_driver = motors.motor_driver:main',
        'motor_driver_ros_interface = motors.motor_driver_ros_interface:main',
        'motor_test = motors.motor_test:main',
        'rhino_params = motors.rhino_params:main',
        'motor_control = motors.motor_control:main'
        ],
    },
)
