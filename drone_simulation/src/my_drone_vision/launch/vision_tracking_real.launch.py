import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Real Tello: driver (no joy) + ArUco detector + controller. Single cmd_vel source: the controller.

def generate_launch_description():
    tello_share = get_package_share_directory('tello_driver')
    driver_launch = os.path.join(tello_share, 'launch', 'driver_only.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(driver_launch)),
        Node(
            package='my_drone_vision',
            executable='aruco_detector',
            name='aruco_detector_node',
            output='screen',
        ),
        Node(
            package='my_drone_vision',
            executable='aruco_controller',
            name='aruco_controller_node',
            output='screen',
        ),
    ])
