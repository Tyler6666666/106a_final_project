from launch import LaunchDescription
from launch_ros.actions import Node

# Tello driver only: camera, cmd_vel, tello_action — no joystick (no extra cmd_vel source).
# Use with my_drone_vision for ArUco tracking. Namespace matches vision nodes (/drone1/...).

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tello_driver',
            executable='tello_driver_main',
            namespace='drone1',
            output='screen',
        ),
    ])
