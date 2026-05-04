#!/usr/bin/env python3
"""
ArUco Marker Detector Node (3D version).
This node detects ArUco markers and publishes full 3D pose with a valid quaternion.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def rvec_to_quaternion(rvec):
    """Convert OpenCV Rodrigues rotation vector to quaternion (x, y, z, w)."""
    theta = float(np.linalg.norm(rvec))
    if theta < 1e-12:
        return 0.0, 0.0, 0.0, 1.0

    axis = (rvec / theta).reshape(3)
    half = theta * 0.5
    sin_half = float(np.sin(half))
    qx = float(axis[0] * sin_half)
    qy = float(axis[1] * sin_half)
    qz = float(axis[2] * sin_half)
    qw = float(np.cos(half))
    return qx, qy, qz, qw


class ArucoDetector3D(Node):
    def __init__(self):
        super().__init__("aruco_detector_3d_node")
        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(Image, "/drone1/image_raw", self.image_cb, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, "/drone1/camera_info", self.info_cb, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(Pose, "/aruco/pose_3d", 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.cam_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.15

    def info_cb(self, msg):
        self.cam_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        if self.cam_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().error(f"CV Bridge Error: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.cam_matrix, self.dist_coeffs
            )

            pose_msg = Pose()
            pose_msg.position.x = float(tvecs[0][0][0])
            pose_msg.position.y = float(tvecs[0][0][1])
            pose_msg.position.z = float(tvecs[0][0][2])

            qx, qy, qz, qw = rvec_to_quaternion(rvecs[0][0])
            pose_msg.orientation.x = qx
            pose_msg.orientation.y = qy
            pose_msg.orientation.z = qz
            pose_msg.orientation.w = qw
            self.pose_pub.publish(pose_msg)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.cam_matrix, self.dist_coeffs, rvecs[0][0], tvecs[0][0], 0.1)

        cv2.imshow("Drone Vision 3D", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
