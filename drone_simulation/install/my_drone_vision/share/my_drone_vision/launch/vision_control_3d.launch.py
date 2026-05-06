from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="my_drone_vision",
                executable="aruco_detector_3d",
                name="aruco_detector_3d_node",
                output="screen",
            ),
            Node(
                package="my_drone_vision",
                executable="aruco_controller_3d",
                name="aruco_controller_3d_node",
                output="screen",
            ),
        ]
    )
