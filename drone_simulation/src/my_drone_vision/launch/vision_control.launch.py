from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动视觉检测节点
        Node(
            package='my_drone_vision',
            executable='aruco_detector',  # 确保你在 setup.py 里注册了这个名字
            name='aruco_detector_node',
            output='screen'
        ),
        # 启动控制节点
        Node(
            package='my_drone_vision',
            executable='aruco_controller', # 确保你在 setup.py 里注册了这个名字
            name='aruco_controller_node',
            output='screen'
        )
    ])