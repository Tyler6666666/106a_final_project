#!/usr/bin/env python3
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction


DEFAULT_CAMERA_MATRIX = np.array(
    [
        [921.170702, 0.0, 459.904354],
        [0.0, 919.018377, 351.238301],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)
DEFAULT_DIST_COEFFS = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.0], dtype=np.float32)
MARKER_POINTS_UNIT = np.array(
    [
        [-0.5, 0.5, 0.0],
        [0.5, 0.5, 0.0],
        [0.5, -0.5, 0.0],
        [-0.5, -0.5, 0.0],
    ],
    dtype=np.float32,
)


class TelloDirectIO(Node):
    def __init__(self):
        super().__init__("tello_direct_io_node")

        self.declare_parameter("pose_topic", "/aruco/pose_3d")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tello_action_service", "/tello_action")
        self.declare_parameter("flight_data_topic", "/flight_data")
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("marker_size", 0.15)
        self.declare_parameter("frame_width", 320) #640 before
        self.declare_parameter("frame_height", 240) #480 before
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("image_topic", "/tello/debug_image")
        self.declare_parameter("image_frame_id", "tello_camera")
        self.declare_parameter("max_lr_rc", 12)
        self.declare_parameter("max_fb_rc", 18)
        self.declare_parameter("max_ud_rc", 12)
        self.declare_parameter("max_yaw_rc", 25)
        self.declare_parameter("rc_send_period_sec", 0.10)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.tello_action_service = self.get_parameter("tello_action_service").value
        self.flight_data_topic = self.get_parameter("flight_data_topic").value
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.image_frame_id = str(self.get_parameter("image_frame_id").value)
        self.max_lr_rc = int(self.get_parameter("max_lr_rc").value)
        self.max_fb_rc = int(self.get_parameter("max_fb_rc").value)
        self.max_ud_rc = int(self.get_parameter("max_ud_rc").value)
        self.max_yaw_rc = int(self.get_parameter("max_yaw_rc").value)
        self.rc_send_period_sec = float(self.get_parameter("rc_send_period_sec").value)

        self.camera_matrix = DEFAULT_CAMERA_MATRIX.copy()
        self.camera_matrix[0, :] *= self.frame_width / 960.0
        self.camera_matrix[1, :] *= self.frame_height / 720.0
        self.dist_coeffs = DEFAULT_DIST_COEFFS.copy()

        self.pose_pub = self.create_publisher(Pose, self.pose_topic, 10)
        self.flight_pub = self.create_publisher(FlightData, self.flight_data_topic, 10)
        self.image_pub = (
            self.create_publisher(Image, self.image_topic, qos_profile_sensor_data)
            if self.publish_debug_image
            else None
        )
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10)
        self.action_srv = self.create_service(TelloAction, self.tello_action_service, self.action_cb)

        self.tello = None
        self.frame_read = None
        self.airborne = False
        self.last_rc = (0, 0, 0, 0)
        self.last_frame_time = 0.0
        self.last_rc_send_time = 0.0
        self.command_lock = threading.Lock()

        self.aruco_dict, self.aruco_params, self.aruco_detector = self.make_detector()
        self.connect_tello()

        self.create_timer(1.0 / 20.0, self.process_frame) #30 fps before
        self.create_timer(1.0, self.publish_flight_data)
        self.get_logger().warn("Using direct djitellopy video/control. Do not run tello_driver at the same time.")

    def connect_tello(self):
        try:
            from djitellopy import Tello
        except ImportError as exc:
            raise RuntimeError("djitellopy is required for tello_direct_io. Install it before running.") from exc

        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f"Tello battery: {self.tello.get_battery()}%")
        self.tello.streamoff()
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

    @staticmethod
    def make_detector():
        aruco = cv2.aruco
        if hasattr(aruco, "getPredefinedDictionary"):
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
        else:
            dictionary = aruco.Dictionary_get(aruco.DICT_6X6_50)

        if hasattr(aruco, "DetectorParameters"):
            parameters = aruco.DetectorParameters()
        else:
            parameters = aruco.DetectorParameters_create()

        detector = None
        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(dictionary, parameters)
        return dictionary, parameters, detector

    @staticmethod
    def clip_rc(value, limit):
        return int(max(-limit, min(limit, round(value))))

    def cmd_vel_cb(self, msg):
        if self.tello is None:
            return

        lr = self.clip_rc(-msg.linear.y * 100.0, self.max_lr_rc)
        fb = self.clip_rc(msg.linear.x * 100.0, self.max_fb_rc)
        ud = self.clip_rc(msg.linear.z * 100.0, self.max_ud_rc)
        yaw = self.clip_rc(-msg.angular.z * 100.0, self.max_yaw_rc)
        if not self.airborne and (lr != 0 or fb != 0 or ud != 0 or yaw != 0):
            if self.last_rc != (0, 0, 0, 0):
                self.last_rc = (0, 0, 0, 0)
                self.send_rc_if_ready(0, 0, 0, 0, force=True)
            return

        self.last_rc = (lr, fb, ud, yaw)
        self.send_rc_if_ready(lr, fb, ud, yaw)

    def send_rc_if_ready(self, lr, fb, ud, yaw, *, force=False):
        now = time.time()
        if not force and now - self.last_rc_send_time < self.rc_send_period_sec:
            return
        if not self.command_lock.acquire(blocking=False):
            return
        try:
            self.tello.send_rc_control(lr, fb, ud, yaw)
            self.last_rc_send_time = now
        finally:
            self.command_lock.release()

    def action_cb(self, request, response):
        if self.tello is None:
            response.rc = response.ERROR_NOT_CONNECTED
            return response

        cmd = request.cmd.strip()
        threading.Thread(target=self.run_tello_action, args=(cmd,), daemon=True).start()
        response.rc = response.OK
        return response

    def run_tello_action(self, cmd):
        try:
            with self.command_lock:
                if cmd == "takeoff":
                    self.tello.takeoff()
                    self.airborne = True
                elif cmd == "land":
                    self.tello.send_rc_control(0, 0, 0, 0)
                    self.tello.land()
                    self.airborne = False
                elif cmd == "emergency":
                    self.tello.emergency()
                    self.airborne = False
                elif cmd.endswith("?"):
                    self.tello.send_read_command(cmd)
                else:
                    self.tello.send_control_command(cmd)
        except Exception as exc:
            self.get_logger().error(f"Tello command failed: {cmd}: {exc}")
            if cmd in ("takeoff", "land", "emergency"):
                self.airborne = False

    def process_frame(self):
        if self.frame_read is None:
            return

        frame = self.frame_read.frame
        if frame is None:
            return

        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        tag, corners, ids = self.detect_marker(frame)
        if tag is not None:
            pose = self.make_pose(tag)
            self.pose_pub.publish(pose)

        if self.publish_debug_image:
            self.draw_debug(frame, tag, corners, ids)
            self.publish_image(frame)

    def detect_marker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return None, corners, ids

        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self.marker_id)[0]
        if len(matches) == 0:
            return None, corners, ids

        index = int(matches[0])
        marker_corners = corners[index].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            MARKER_POINTS_UNIT * self.marker_size,
            marker_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None, corners, ids

        rotation, _ = cv2.Rodrigues(rvec)
        yaw = np.arctan2(-rotation[2, 0], np.sqrt(rotation[2, 1] ** 2 + rotation[2, 2] ** 2))
        cx = int(np.mean(marker_corners[:, 0]))
        cy = int(np.mean(marker_corners[:, 1]))
        return (cx, cy, rvec, tvec.reshape(3), float(yaw)), corners, ids

    @staticmethod
    def make_pose(tag):
        _, _, _, tvec, yaw = tag
        pose = Pose()
        pose.position.x = float(tvec[0])
        pose.position.y = float(tvec[1])
        pose.position.z = float(tvec[2])
        pose.orientation.z = yaw
        return pose

    def draw_debug(self, frame, tag, corners, ids):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        if tag is not None:
            cx, cy, rvec, tvec, _ = tag
            cv2.circle(frame, (cx, cy), 8, (0, 255, 0), cv2.FILLED)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size * 0.5)
            cv2.putText(
                frame,
                f"id={self.marker_id} z={float(tvec[2]):.2f}m rc={self.last_rc}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def publish_image(self, frame):
        if self.image_pub is None:
            return
        frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.image_frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        self.image_pub.publish(msg)

    def publish_flight_data(self):
        if self.tello is None:
            return
        msg = FlightData()
        msg.header.stamp = self.get_clock().now().to_msg()
        try:
            state = self.tello.get_current_state()
            msg.bat = int(state.get("bat", self.tello.get_battery()))
            msg.h = int(state.get("h", 0))
            msg.tof = int(state.get("tof", 0))
            msg.templ = int(state.get("templ", 0))
            msg.temph = int(state.get("temph", 0))
            msg.time = int(state.get("time", 0))
            msg.sdk = FlightData.SDK_UNKNOWN
            msg.raw = str(state)
        except Exception as exc:
            self.get_logger().warn(f"Failed to read Tello state: {exc}")
        self.flight_pub.publish(msg)

    def shutdown_tello(self):
        if self.tello is None:
            return
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.streamoff()
            self.tello.end()
        except Exception as exc:
            self.get_logger().warn(f"Tello shutdown warning: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = TelloDirectIO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_tello()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
