from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation.
# Namespace matches my_drone_vision and tello_gazebo (e.g. /drone1/cmd_vel, /drone1/image_raw).

def generate_launch_description():
    ns = 'drone1'
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            namespace=ns,
            output='screen',
        ),
        Node(
            package='tello_driver',
            executable='tello_joy_main',
            namespace=ns,
            output='screen',
        ),
        Node(
            package='tello_driver',
            executable='tello_driver_main',
            namespace=ns,
            output='screen',
        ),
    ])

