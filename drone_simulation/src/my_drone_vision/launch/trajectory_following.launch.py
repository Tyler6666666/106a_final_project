from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Same hardware / ArUco stack as real_follow; controller runs trajectory recording + replay mission."""
    return LaunchDescription(
        [
            Node(
                package="my_drone_vision",
                executable="tello_direct_io",
                name="tello_direct_io_node",
                output="screen",
                parameters=[
                    {
                        "pose_topic": "/aruco/pose_3d",
                        "cmd_vel_topic": "/cmd_vel",
                        "tello_action_service": "/tello_action",
                        "flight_data_topic": "/flight_data",
                        "marker_id": 0,
                        "marker_size": 0.15,
                        "frame_width": 320,
                        "frame_height": 240,
                        "publish_debug_image": True,
                        "image_topic": "/tello/debug_image",
                        "image_frame_id": "tello_camera",
                        "max_lr_rc": 12,
                        "max_fb_rc": 18,
                        "max_ud_rc": 12,
                        "max_yaw_rc": 25,
                        "rc_send_period_sec": 0.10,
                    }
                ],
            ),
            Node(
                package="my_drone_vision",
                executable="trajectory_follow_controller",
                name="trajectory_follow_controller_node",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": "/cmd_vel",
                        "tello_action_service": "/tello_action",
                        "pose_topic": "/aruco/pose_3d",
                        "auto_takeoff": True,
                        "follow_after_takeoff_sec": 2.0,
                        "target_dist": 0.60,
                        "require_tag_before_takeoff": True,
                        "tag_confirm_sec": 5.0,
                        # Mission timing (adjust here)
                        "memory_sec": 30.0,
                        "wait_before_replay_sec": 3.0,
                        # Default: do not RTL on brief tag occlusion during FOLLOW/REPLAY
                        "follow_land_on_tag_loss": False,
                        "tag_loss_land_sec": 5.0,
                        "enable_search": False,
                        "search_speed": 0.25,
                        "search_duration": 8.0,
                        "max_forward_speed": 0.18,
                        "max_side_speed": 0.12,
                        "max_vertical_speed": 0.12,
                        "max_yaw_speed": 0.25,
                        "kp_side": 0.25,
                        "kd_side": 0.03,
                        "kp_yaw": 0.80,
                        "kd_yaw": 0.05,
                        "kp_fwd": 0.45,
                        "kd_fwd": 0.08,
                        "kp_z": 0.55,
                        "yaw_from_x_gain": 0.75,
                        "anticipate_gain": 0.0,
                    }
                ],
            ),
        ]
    )
