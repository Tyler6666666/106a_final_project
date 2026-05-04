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
            output='screen',
            parameters=[{
                'auto_takeoff': True,
                'takeoff_height': 0.5,
                'follow_after_takeoff_sec': 1.0,
                'target_dist': 0.45,
                'land_on_tag_loss': True,
                'tag_loss_land_sec': 1.0,
                'enable_search': False,
            }]
        )
    ])
