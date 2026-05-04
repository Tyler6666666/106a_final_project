#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


OUT = "/mnt/c/Users/aaron/Desktop/cs106/106a_final_project/drone_camera_live.jpg"
ANN = "/mnt/c/Users/aaron/Desktop/cs106/106a_final_project/drone_camera_live_aruco.jpg"


class OneFrameArucoProbe(Node):
    def __init__(self):
        super().__init__("one_frame_aruco_probe")
        self.frame = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.create_subscription(Image, "/image_raw", self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera_info", self.info_cb, qos_profile_sensor_data)

    def info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_cb(self, msg):
        encoding = msg.encoding.lower()
        channels = 3 if encoding in ("bgr8", "rgb8") else 1
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
        arr = arr[:, :msg.width * channels]

        if channels == 1:
            frame = arr.reshape((msg.height, msg.width))
        else:
            frame = arr.reshape((msg.height, msg.width, channels))
            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        self.frame = np.ascontiguousarray(frame)


def main():
    rclpy.init()
    node = OneFrameArucoProbe()
    deadline = time.time() + 6.0
    while rclpy.ok() and time.time() < deadline and node.frame is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.frame is None:
        print("NO_FRAME")
    else:
        frame = node.frame
        cv2.imwrite(OUT, frame)
        gray = frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        else:
            dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

        if hasattr(cv2.aruco, "DetectorParameters"):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, "ArucoDetector"):
            corners, ids, rejected = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imwrite(ANN, frame)
        print(
            "FRAME",
            frame.shape,
            "IDS",
            None if ids is None else ids.flatten().tolist(),
            "REJECTED",
            len(rejected),
            "OUT",
            OUT,
            "ANN",
            ANN,
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
