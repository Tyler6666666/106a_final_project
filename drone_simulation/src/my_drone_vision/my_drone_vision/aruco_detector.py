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
import cv2
import numpy as np

class ArucoDetector(Node):
    """
    Detects ArUco markers and publishes their 3D position and Yaw angle.
    """
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('marker_size', 0.15)
        image_topic = self.get_parameter('image_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = self.get_parameter('marker_size').value

        # Subscriptions for raw image and camera intrinsics
        self.img_sub = self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.info_cb, qos_profile_sensor_data)
        
        # Publisher for the calculated 3D pose
        self.pose_pub = self.create_publisher(Pose, '/aruco/pose_3d', 10)

        # ArUco dictionary and parameters initialization
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.aruco_detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera parameters (populated via info_cb)
        self.cam_matrix = None
        self.dist_coeffs = None
        
        # Actual physical side length of the black ArUco marker in meters.

    def ros_image_to_bgr(self, msg):
        """
        Convert a ROS Image message to a BGR OpenCV image without cv_bridge.
        This keeps the detector usable in WSL when cv_bridge and NumPy ABI differ.
        """
        encoding = msg.encoding.lower()
        if encoding in ('bgr8', 'rgb8'):
            channels = 3
        elif encoding in ('mono8', '8uc1'):
            channels = 1
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        arr = arr.reshape((msg.height, msg.step))
        arr = arr[:, :msg.width * channels]

        if channels == 1:
            return np.ascontiguousarray(arr.reshape((msg.height, msg.width)))

        frame = arr.reshape((msg.height, msg.width, channels))
        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

    def estimate_marker_pose(self, marker_corners):
        """
        Estimate marker pose with solvePnP so the node works with OpenCV builds
        that do not ship cv2.aruco.estimatePoseSingleMarkers.
        """
        if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners], self.marker_size, self.cam_matrix, self.dist_coeffs
            )
            return rvecs[0][0], tvecs[0][0]

        half = self.marker_size / 2.0
        object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)
        image_points = marker_corners.reshape((4, 2)).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.cam_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            raise RuntimeError("solvePnP failed for detected ArUco marker")
        return rvec.reshape(3), tvec.reshape(3)

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
            frame = self.ros_image_to_bgr(msg)
        except Exception as e: 
            self.get_logger().error(f"Image Conversion Error: {e}")
            return

        if len(frame.shape) == 2:
            gray = frame
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect markers in the image
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            try:
                rvec, tvec = self.estimate_marker_pose(corners[0])
            except Exception as e:
                self.get_logger().error(f"Pose Estimation Error: {e}")
                return
            
            # [CORE MODIFICATION] Calculate Yaw directly from the rotation vector.
            # In the camera coordinate system, rotation around the Y-axis represents the Yaw.
            R, _ = cv2.Rodrigues(rvec)
            
            # Extract the angle (in radians) around the Y-axis
            marker_yaw = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))

            # Populate the Pose message
            pose_msg = Pose()
            pose_msg.position.x = float(tvec[0])
            pose_msg.position.y = float(tvec[1])
            pose_msg.position.z = float(tvec[2])
            
            # Hack: Borrow orientation.z to pass the calculated yaw.
            # This bypasses the need for complex quaternion conversions in this specific use-case.
            pose_msg.orientation.z = float(marker_yaw)
            
            self.pose_pub.publish(pose_msg)

        # Image display is disabled for Docker/WSL headless operation.

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
