#!/usr/bin/env python3
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class ArucoLiveView(Node):
    def __init__(self):
        super().__init__("aruco_live_view")
        self.cam_matrix = None
        self.dist_coeffs = None
        self.marker_size = 0.15
        self.frame_count = 0
        self.detect_count = 0
        self.last_detect = 0.0

        self.out_dir = Path("/mnt/c/Users/aaron/Desktop/cs106/106a_final_project/artifacts/live")
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.tmp_image = self.out_dir / "tello_aruco_live.tmp.jpg"
        self.out_image = self.out_dir / "tello_aruco_live.jpg"
        self.tmp_status = self.out_dir / "status.tmp.json"
        self.out_status = self.out_dir / "status.json"

        if hasattr(cv2.aruco, "getPredefinedDictionary"):
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        else:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.create_subscription(Image, "/image_raw", self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera_info", self.info_cb, qos_profile_sensor_data)

    def info_cb(self, msg):
        self.cam_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def ros_image_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "rgb8"):
            channels = 3
        elif encoding in ("mono8", "8uc1"):
            channels = 1
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.step))
        arr = arr[:, :msg.width * channels]
        if channels == 1:
            return np.ascontiguousarray(arr.reshape((msg.height, msg.width)))

        frame = arr.reshape((msg.height, msg.width, channels))
        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(frame)

    def estimate_marker_pose(self, marker_corners):
        if self.cam_matrix is None or self.dist_coeffs is None:
            return None, None

        if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners], self.marker_size, self.cam_matrix, self.dist_coeffs
            )
            return rvecs[0][0], tvecs[0][0]

        half = self.marker_size / 2.0
        object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
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
            return None, None
        return rvec.reshape(3), tvec.reshape(3)

    def image_cb(self, msg):
        self.frame_count += 1
        try:
            frame = self.ros_image_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        gray = frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        draw_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if len(frame.shape) == 2 else frame.copy()

        status = {
            "frames": self.frame_count,
            "detected": False,
            "ids": [],
            "timestamp": time.time(),
        }

        if ids is not None and len(ids) > 0:
            self.detect_count += 1
            self.last_detect = time.time()
            flat_ids = ids.flatten().tolist()
            status["detected"] = True
            status["ids"] = flat_ids
            cv2.aruco.drawDetectedMarkers(draw_frame, corners, ids)

            rvec, tvec = self.estimate_marker_pose(corners[0])
            if rvec is not None and tvec is not None:
                R, _ = cv2.Rodrigues(rvec)
                marker_yaw = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))

                cv2.drawFrameAxes(draw_frame, self.cam_matrix, self.dist_coeffs, rvec, tvec, 0.08)
                status["pose_m"] = {
                    "x": float(tvec[0]),
                    "y": float(tvec[1]),
                    "z": float(tvec[2]),
                    "yaw": float(marker_yaw),
                }

        age = time.time() - self.last_detect if self.last_detect else None
        banner = "ARUCO DETECTED" if status["detected"] else "NO TAG DETECTED"
        color = (0, 220, 0) if status["detected"] else (0, 0, 255)
        cv2.rectangle(draw_frame, (0, 0), (draw_frame.shape[1], 44), (0, 0, 0), -1)
        cv2.putText(draw_frame, banner, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.85, color, 2, cv2.LINE_AA)

        detail = f"DICT_6X6_50 id0 marker=15cm frames={self.frame_count}"
        if status["detected"]:
            detail += f" ids={status['ids']}"
            if "pose_m" in status:
                detail += f" z={status['pose_m']['z']:.2f}m"
        elif age is not None:
            detail += f" last_seen={age:.1f}s"
        cv2.putText(
            draw_frame,
            detail,
            (14, draw_frame.shape[0] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
        )

        cv2.imwrite(str(self.tmp_image), draw_frame)
        self.tmp_image.replace(self.out_image)
        self.tmp_status.write_text(json.dumps(status, indent=2))
        self.tmp_status.replace(self.out_status)


def main():
    rclpy.init()
    node = ArucoLiveView()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
