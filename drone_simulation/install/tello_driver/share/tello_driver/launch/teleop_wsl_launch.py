from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tello_driver_params = [{
        'drone_ip': '192.168.10.2',
        'drone_port': 18889,
        'command_port': 38065,
        'camera_info_path': '/mnt/c/Users/aaron/Desktop/cs106/106a_final_project/drone_simulation/src/tello_ros/tello_driver/cfg/camera_info.yaml',
    }]

    return LaunchDescription([
        Node(package='joy', executable='joy_node', output='screen'),
        Node(package='tello_driver', executable='tello_joy_main', output='screen'),
        Node(package='tello_driver', executable='tello_driver_main', output='screen',
             parameters=tello_driver_params),
    ])
