#!/usr/bin/env python3
"""
ArUco Marker Detector Node.
This node subscribes to a drone's camera stream and camera info, detects 
ArUco markers, estimates their 3D pose, and calculates the yaw angle 
relative to the camera. It publishes the target pose for the controller.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose 
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    """
    Detects ArUco markers and publishes their 3D position and Yaw angle.
    """
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()
        
        # Subscriptions for raw image and camera intrinsics
        self.img_sub = self.create_subscription(Image, '/drone1/image_raw', self.image_cb, 10)
        # Match tello_driver camera_info publisher (SensorDataQoS / BEST_EFFORT).
        self.info_sub = self.create_subscription(
            CameraInfo, '/drone1/camera_info', self.info_cb, qos_profile_sensor_data)
        
        # Publisher for the calculated 3D pose
        self.pose_pub = self.create_publisher(Pose, '/aruco/pose_3d', 10)

        # ArUco dictionary and parameters initialization
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Camera parameters (populated via info_cb)
        self.cam_matrix = None
        self.dist_coeffs = None
        
        # Actual physical size of the marker in meters
        self.marker_size = 0.15 

    def info_cb(self, msg):
        """
        Callback to extract and store camera intrinsic parameters.
        """
        self.cam_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        """
        Callback to process incoming image frames, detect markers, 
        calculate pose/yaw, and publish the results.
        """
        # Wait until camera intrinsics are available
        if self.cam_matrix is None: 
            return
            
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e: 
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers in the image
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimate pose for each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.cam_matrix, self.dist_coeffs
            )
            
            # [CORE MODIFICATION] Calculate Yaw directly from the rotation vector.
            # In the camera coordinate system, rotation around the Y-axis represents the Yaw.
            R, _ = cv2.Rodrigues(rvecs[0][0])
            
            # Extract the angle (in radians) around the Y-axis
            marker_yaw = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))

            # Populate the Pose message
            pose_msg = Pose()
            pose_msg.position.x = float(tvecs[0][0][0])
            pose_msg.position.y = float(tvecs[0][0][1])
            pose_msg.position.z = float(tvecs[0][0][2])
            
            # Hack: Borrow orientation.z to pass the calculated yaw.
            # This bypasses the need for complex quaternion conversions in this specific use-case.
            pose_msg.orientation.z = float(marker_yaw)
            
            self.pose_pub.publish(pose_msg)

            # Visual Enhancements: Draw marker boundaries and coordinate axes
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.cam_matrix, self.dist_coeffs, rvecs[0][0], tvecs[0][0], 0.1)
        
        # Display the debug window
        cv2.imshow("Drone Vision", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
