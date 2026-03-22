#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose 
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()
        self.img_sub = self.create_subscription(Image, '/drone1/image_raw', self.image_cb, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/drone1/camera_info', self.info_cb, 10)
        self.pose_pub = self.create_publisher(Pose, '/aruco/pose_3d', 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.cam_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.15 

    def info_cb(self, msg):
        self.cam_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        if self.cam_matrix is None: return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.cam_matrix, self.dist_coeffs)
            
            # 【核心修改】直接从旋转向量计算 Yaw
            # 在相机坐标系中，绕 Y 轴的旋转才是我们要的 Yaw
            R, _ = cv2.Rodrigues(rvecs[0][0])
            # 提取绕 Y 轴的弧度
            marker_yaw = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))

            pose_msg = Pose()
            pose_msg.position.x = float(tvecs[0][0][0])
            pose_msg.position.y = float(tvecs[0][0][1])
            pose_msg.position.z = float(tvecs[0][0][2])
            
            # 临时借用 orientation.z 来传递算好的 yaw，绕开四元数转换的坑
            pose_msg.orientation.z = float(marker_yaw)
            self.pose_pub.publish(pose_msg)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.cam_matrix, self.dist_coeffs, rvecs[0][0], tvecs[0][0], 0.1)
        
        cv2.imshow("Drone Vision", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ArucoDetector())
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()