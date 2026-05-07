#!/usr/bin/env python3
import os
os.environ.setdefault("YOLO_OFFLINE", "True")
os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/Ultralytics")

import threading
import time
from pathlib import Path

import cv2
import rclpy
from geometry_msgs.msg import Pose, Twist
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from tello_msgs.srv import TelloAction


class OpenCvFrameSource:
    def __init__(self, source, frame_width, frame_height):
        if source.isdigit():
            self.capture = cv2.VideoCapture(int(source))
        else:
            self.capture = cv2.VideoCapture(source)

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        if not self.capture.isOpened():
            raise RuntimeError(
                f"Could not open video_source={source}. Use camera index like '0', "
                "a stream URL, or 'tello' for djitellopy."
            )

    def read(self):
        return self.capture.read()

    def close(self):
        self.capture.release()


class TelloFrameSource:
    def __init__(self, logger):
        try:
            from djitellopy import Tello
        except ImportError as exc:
            raise RuntimeError("djitellopy is required when video_source='tello'. Install it before running.") from exc

        self.logger = logger
        self.tello = Tello()
        self.tello.connect()
        self.tello.for_back_velocity = 0
        self.tello.left_right_velocity = 0
        self.tello.up_down_velocity = 0
        self.tello.yaw_velocity = 0
        self.tello.speed = 0
        self.logger.info(f"Tello battery: {self.tello.get_battery()}%")
        self.tello.streamoff()
        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()

    def read(self):
        if self.frame_read is None:
            return False, None
        return True, self.frame_read.frame

    def send_rc_control(self, lr, fb, ud, yaw):
        self.tello.send_rc_control(lr, fb, ud, yaw)

    def takeoff(self):
        self.tello.takeoff()

    def land(self):
        self.tello.send_rc_control(0, 0, 0, 0)
        self.tello.land()

    def close(self):
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.streamoff()
            self.tello.end()
        except Exception as exc:
            self.logger.warn(f"Tello shutdown warning: {exc}")


class HaarFaceBackend:
    def __init__(self):
        cascade_path = self.find_cascade_path()
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f"Failed to load OpenCV face cascade: {cascade_path}")
        self.name = "haar"

    @staticmethod
    def find_cascade_path():
        candidates = []
        if hasattr(cv2, "data") and hasattr(cv2.data, "haarcascades"):
            candidates.append(os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml"))
        candidates.extend([
            "haarcascade_frontalface_default.xml",
            "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
            "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
        ])
        for path in candidates:
            if path and os.path.exists(path):
                return path
        return candidates[-2]

    def detect_largest_face(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(45, 45),
            flags=cv2.CASCADE_SCALE_IMAGE,
        )
        if len(faces) == 0:
            return None
        x, y, w, h = max(faces, key=lambda rect: rect[2] * rect[3])
        return (int(x), int(y), int(w), int(h), 1.0)


class YoloFaceBackend:
    def __init__(self, model_path, confidence, image_size, class_id, logger):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for detector_backend='yolo'. "
                "Install it with: python3 -m pip install --user ultralytics"
            ) from exc

        self.name = "yolo"
        self.confidence = float(confidence)
        self.image_size = int(image_size)
        self.class_id = int(class_id)
        self.model_path = self.resolve_model_path(model_path)
        if not self.model_path.exists():
            raise RuntimeError(
                f"YOLO face model not found: {self.model_path}. "
                "Place a YOLOv8 face model such as yolov8n-face.pt there, "
                "or set the yolo_model_path launch parameter."
            )

        logger.info(f"Loading YOLOv8 face model: {self.model_path}")
        self.model = YOLO(str(self.model_path))

    @staticmethod
    def resolve_model_path(model_path):
        expanded = Path(os.path.expanduser(str(model_path)))
        if expanded.is_absolute():
            return expanded

        candidates = [Path.cwd() / expanded]
        try:
            share_dir = Path(get_package_share_directory("my_drone_vision"))
            candidates.append(share_dir / expanded)
            candidates.append(share_dir / "models" / expanded.name)
        except PackageNotFoundError:
            pass
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[0]

    def detect_largest_face(self, frame):
        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=self.image_size,
            verbose=False,
        )
        if not results:
            return None

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        best = None
        best_area = 0.0
        for box in boxes:
            cls = int(box.cls[0].item()) if box.cls is not None else -1
            if self.class_id >= 0 and cls != self.class_id:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            area = w * h
            if area > best_area:
                confidence = float(box.conf[0].item()) if box.conf is not None else 0.0
                best = (int(x1), int(y1), int(w), int(h), confidence)
                best_area = area
        return best


class FaceDetector(Node):
    def __init__(self):
        super().__init__("face_detector_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("face_pose_topic", "/face/pose")
        self.declare_parameter("tello_action_service", "/tello_action")
        self.declare_parameter("video_source", "tello")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("show_window", True)
        self.declare_parameter("detector_backend", "yolo")
        self.declare_parameter("yolo_model_path", "src/my_drone_vision/models/yolov8n-face.pt")
        self.declare_parameter("yolo_confidence", 0.45)
        self.declare_parameter("yolo_image_size", 640)
        self.declare_parameter("yolo_class_id", -1)
        self.declare_parameter("publish_cmd_vel", False)
        self.declare_parameter("direct_tello_control", False)
        self.declare_parameter("auto_takeoff", False)
        self.declare_parameter("require_face_before_takeoff", True)
        self.declare_parameter("face_confirm_sec", 2.0)
        self.declare_parameter("follow_after_takeoff_sec", 2.0)
        self.declare_parameter("land_on_face_loss", True)
        self.declare_parameter("face_loss_land_sec", 5.0)
        self.declare_parameter("target_face_area_ratio", 0.10)
        self.declare_parameter("deadband_x", 0.08)
        self.declare_parameter("deadband_y", 0.08)
        self.declare_parameter("deadband_area", 0.025)
        self.declare_parameter("max_forward_speed", 0.18)
        self.declare_parameter("max_vertical_speed", 0.15)
        self.declare_parameter("max_yaw_speed", 0.35)
        self.declare_parameter("kp_forward", 0.85)
        self.declare_parameter("kp_vertical", 0.45)
        self.declare_parameter("kp_yaw", 0.70)
        self.declare_parameter("lost_timeout_sec", 0.6)
        self.declare_parameter("search_when_lost", False)
        self.declare_parameter("search_yaw_speed", 0.18)
        self.declare_parameter("provide_tello_io", True)
        self.declare_parameter("max_lr_rc", 15)
        self.declare_parameter("max_fb_rc", 18)
        self.declare_parameter("max_ud_rc", 15)
        self.declare_parameter("max_yaw_rc", 35)
        self.declare_parameter("rc_send_period_sec", 0.10)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.face_pose_topic = self.get_parameter("face_pose_topic").value
        self.tello_action_service = self.get_parameter("tello_action_service").value
        self.video_source = str(self.get_parameter("video_source").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.detector_backend = str(self.get_parameter("detector_backend").value).lower()
        self.yolo_model_path = self.get_parameter("yolo_model_path").value
        self.yolo_confidence = float(self.get_parameter("yolo_confidence").value)
        self.yolo_image_size = int(self.get_parameter("yolo_image_size").value)
        self.yolo_class_id = int(self.get_parameter("yolo_class_id").value)
        self.publish_cmd_vel = bool(self.get_parameter("publish_cmd_vel").value)
        self.direct_tello_control = bool(self.get_parameter("direct_tello_control").value)
        self.auto_takeoff_enabled = bool(self.get_parameter("auto_takeoff").value)
        self.require_face_before_takeoff = bool(self.get_parameter("require_face_before_takeoff").value)
        self.face_confirm_sec = float(self.get_parameter("face_confirm_sec").value)
        self.follow_after_takeoff_sec = float(self.get_parameter("follow_after_takeoff_sec").value)
        self.land_on_face_loss = bool(self.get_parameter("land_on_face_loss").value)
        self.face_loss_land_sec = float(self.get_parameter("face_loss_land_sec").value)
        self.target_face_area_ratio = float(self.get_parameter("target_face_area_ratio").value)
        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)
        self.deadband_area = float(self.get_parameter("deadband_area").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_vertical_speed = float(self.get_parameter("max_vertical_speed").value)
        self.max_yaw_speed = float(self.get_parameter("max_yaw_speed").value)
        self.kp_forward = float(self.get_parameter("kp_forward").value)
        self.kp_vertical = float(self.get_parameter("kp_vertical").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.lost_timeout_sec = float(self.get_parameter("lost_timeout_sec").value)
        self.search_when_lost = bool(self.get_parameter("search_when_lost").value)
        self.search_yaw_speed = float(self.get_parameter("search_yaw_speed").value)
        self.provide_tello_io = bool(self.get_parameter("provide_tello_io").value)
        self.max_lr_rc = int(self.get_parameter("max_lr_rc").value)
        self.max_fb_rc = int(self.get_parameter("max_fb_rc").value)
        self.max_ud_rc = int(self.get_parameter("max_ud_rc").value)
        self.max_yaw_rc = int(self.get_parameter("max_yaw_rc").value)
        self.rc_send_period_sec = float(self.get_parameter("rc_send_period_sec").value)

        self.face_pub = self.create_publisher(Pose, self.face_pose_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.detector = self.make_detector()

        self.frame_source = self.open_frame_source(self.video_source)
        if self.direct_tello_control and not isinstance(self.frame_source, TelloFrameSource):
            raise RuntimeError("direct_tello_control requires video_source='tello'.")
        if self.auto_takeoff_enabled and not isinstance(self.frame_source, TelloFrameSource):
            raise RuntimeError("auto_takeoff requires video_source='tello'.")
        if self.provide_tello_io and not isinstance(self.frame_source, TelloFrameSource):
            raise RuntimeError("provide_tello_io requires video_source='tello'.")

        self.start_time = time.time()
        self.last_seen = 0.0
        self.prev_face_time = None
        self.confirmed_face_time = 0.0
        self.last_detection_log_time = 0.0
        self.last_takeoff_status_log = 0.0
        self.had_face = False
        self.has_sent_takeoff = False
        self.takeoff_sent_time = None
        self.follow_mode_started = False
        self.follow_mode_logged = False
        self.has_sent_land = False
        self.airborne = False
        self.last_rc_send_time = 0.0
        self.command_lock = threading.Lock()

        if self.provide_tello_io:
            self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10)
            self.action_srv = self.create_service(TelloAction, self.tello_action_service, self.action_cb)

        self.create_timer(1.0 / 30.0, self.process_frame)
        self.create_timer(0.5, self.auto_takeoff)
        self.get_logger().info(
            f"Face detector started on video_source={self.video_source}; "
            f"backend={self.detector.name}; publishing pose={self.face_pose_topic}, cmd_vel={self.cmd_vel_topic}."
        )

    def make_detector(self):
        if self.detector_backend == "haar":
            return HaarFaceBackend()
        if self.detector_backend == "yolo":
            return YoloFaceBackend(
                self.yolo_model_path,
                self.yolo_confidence,
                self.yolo_image_size,
                self.yolo_class_id,
                self.get_logger(),
            )
        raise RuntimeError("Unsupported detector_backend. Use 'yolo' or 'haar'.")

    def open_frame_source(self, source):
        if source.lower() == "tello":
            return TelloFrameSource(self.get_logger())
        return OpenCvFrameSource(source, self.frame_width, self.frame_height)

    @staticmethod
    def clip(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def apply_deadband(value, deadband):
        return 0.0 if abs(value) < deadband else value

    def process_frame(self):
        self.update_follow_mode()
        ok, frame = self.frame_source.read()
        if not ok or frame is None:
            self.publish_lost_command()
            if self.should_land_on_face_loss():
                self.land_once("Video frame lost after face tracking; landing.")
            return

        frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        face = self.detect_largest_face(frame)

        if face is not None:
            self.update_face_seen_time()
            self.had_face = True
            pose = self.make_face_pose(face)
            self.face_pub.publish(pose)
            if self.can_follow():
                self.publish_or_send_cmd(self.make_tracking_cmd(pose))
            self.log_detection("tracking")
        else:
            self.reset_face_confirmation_if_stale()
            self.publish_lost_command()
            self.log_detection("lost")
            if self.should_land_on_face_loss():
                self.land_once("Face lost; landing.")

        if self.show_window:
            self.draw_debug(frame, face)
            cv2.imshow("Face Detector", frame)
            cv2.waitKey(1)

    def update_face_seen_time(self):
        now = time.time()
        if self.prev_face_time is None or now - self.prev_face_time > 1.5:
            self.confirmed_face_time = 0.0
        else:
            self.confirmed_face_time += min(now - self.prev_face_time, 0.25)
        self.prev_face_time = now
        self.last_seen = now

    def reset_face_confirmation_if_stale(self):
        if self.prev_face_time is not None and time.time() - self.prev_face_time > 1.5:
            self.prev_face_time = None
            self.confirmed_face_time = 0.0

    def can_follow(self):
        if not self.publish_cmd_vel:
            return False
        if self.has_sent_land:
            return False
        if self.auto_takeoff_enabled and not self.follow_mode_started:
            self.publish_or_send_cmd(Twist())
            return False
        return True

    def detect_largest_face(self, frame):
        return self.detector.detect_largest_face(frame)

    def make_face_pose(self, face):
        x, y, w, h = face[:4]
        cx = x + w / 2.0
        cy = y + h / 2.0
        area_ratio = (w * h) / float(self.frame_width * self.frame_height)

        pose = Pose()
        pose.position.x = (cx - self.frame_width / 2.0) / (self.frame_width / 2.0)
        pose.position.y = (cy - self.frame_height / 2.0) / (self.frame_height / 2.0)
        pose.position.z = area_ratio
        pose.orientation.z = float(w) / max(float(h), 1.0)
        return pose

    def make_tracking_cmd(self, pose):
        err_x = self.apply_deadband(pose.position.x, self.deadband_x)
        err_y = self.apply_deadband(pose.position.y, self.deadband_y)
        err_area = self.apply_deadband(self.target_face_area_ratio - pose.position.z, self.deadband_area)

        cmd = Twist()
        cmd.angular.z = self.clip(-self.kp_yaw * err_x, self.max_yaw_speed)
        cmd.linear.z = self.clip(-self.kp_vertical * err_y, self.max_vertical_speed)
        cmd.linear.x = self.clip(self.kp_forward * err_area, self.max_forward_speed)
        return cmd

    def publish_lost_command(self):
        if not self.publish_cmd_vel:
            return
        if self.has_sent_land:
            return

        now = time.time()
        cmd = Twist()
        if (
            self.search_when_lost
            and self.had_face
            and now - self.last_seen >= self.lost_timeout_sec
            and (not self.auto_takeoff_enabled or self.follow_mode_started)
        ):
            cmd.angular.z = self.search_yaw_speed
        self.publish_or_send_cmd(cmd)

    def publish_or_send_cmd(self, cmd):
        self.cmd_pub.publish(cmd)
        if self.direct_tello_control:
            fb = self.clip_rc(cmd.linear.x * 100.0, self.max_fb_rc)
            ud = self.clip_rc(cmd.linear.z * 100.0, self.max_ud_rc)
            yaw = self.clip_rc(-cmd.angular.z * 100.0, self.max_yaw_rc)
            self.frame_source.send_rc_control(0, fb, ud, yaw)

    def cmd_vel_cb(self, msg):
        if not self.provide_tello_io:
            return
        lr = self.clip_rc(-msg.linear.y * 100.0, self.max_lr_rc)
        fb = self.clip_rc(msg.linear.x * 100.0, self.max_fb_rc)
        ud = self.clip_rc(msg.linear.z * 100.0, self.max_ud_rc)
        yaw = self.clip_rc(-msg.angular.z * 100.0, self.max_yaw_rc)

        if not self.airborne and (lr != 0 or fb != 0 or ud != 0 or yaw != 0):
            self.send_rc_if_ready(0, 0, 0, 0, force=True)
            return
        self.send_rc_if_ready(lr, fb, ud, yaw)

    def send_rc_if_ready(self, lr, fb, ud, yaw, *, force=False):
        now = time.time()
        if not force and now - self.last_rc_send_time < self.rc_send_period_sec:
            return
        if not self.command_lock.acquire(blocking=False):
            return
        try:
            self.frame_source.send_rc_control(lr, fb, ud, yaw)
            self.last_rc_send_time = now
        finally:
            self.command_lock.release()

    def action_cb(self, request, response):
        if not self.provide_tello_io:
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
                    self.frame_source.takeoff()
                    self.airborne = True
                elif cmd == "land":
                    self.frame_source.land()
                    self.airborne = False
                elif cmd == "emergency":
                    self.frame_source.tello.emergency()
                    self.airborne = False
                elif cmd.endswith("?"):
                    self.frame_source.tello.send_read_command(cmd)
                else:
                    self.frame_source.tello.send_control_command(cmd)
        except Exception as exc:
            self.get_logger().error(f"Tello command failed: {cmd}: {exc}")
            if cmd in ("takeoff", "land", "emergency"):
                self.airborne = False

    def auto_takeoff(self):
        if not self.auto_takeoff_enabled or self.has_sent_takeoff or self.has_sent_land:
            return

        now = time.time()
        if now - self.start_time <= 2.0:
            return
        if self.require_face_before_takeoff:
            if now - self.last_seen > 0.75:
                self.log_takeoff_status("Waiting for stable face before takeoff.")
                self.prev_face_time = None
                self.confirmed_face_time = 0.0
                return
            if self.confirmed_face_time < self.face_confirm_sec:
                self.log_takeoff_status(
                    f"Face confirmed for {self.confirmed_face_time:.1f}/{self.face_confirm_sec:.1f}s before takeoff."
                )
                return

        self.get_logger().warn("Face confirmed; taking off.")
        try:
            self.frame_source.takeoff()
        except Exception as exc:
            self.get_logger().error(f"Tello takeoff failed: {exc}")
            return

        self.has_sent_takeoff = True
        self.takeoff_sent_time = time.time()
        self.get_logger().warn(
            f"Takeoff complete; entering face follow mode after {self.follow_after_takeoff_sec:.1f}s."
        )

    def log_takeoff_status(self, message):
        now = time.time()
        if now - self.last_takeoff_status_log >= 1.0:
            self.get_logger().warn(message)
            self.last_takeoff_status_log = now

    def update_follow_mode(self):
        if not self.auto_takeoff_enabled:
            self.follow_mode_started = True
            return
        if not self.has_sent_takeoff or self.takeoff_sent_time is None:
            return
        if self.follow_mode_started:
            return
        elapsed = time.time() - self.takeoff_sent_time
        if elapsed >= self.follow_after_takeoff_sec:
            self.follow_mode_started = True
            if not self.follow_mode_logged:
                self.get_logger().warn("Entering face follow mode.")
                self.follow_mode_logged = True

    def should_land_on_face_loss(self):
        if not self.land_on_face_loss or self.has_sent_land:
            return False
        if self.auto_takeoff_enabled and not self.follow_mode_started:
            return False
        if not self.had_face:
            return False
        return time.time() - self.last_seen >= self.face_loss_land_sec

    def land_once(self, reason):
        if self.has_sent_land:
            return
        self.has_sent_land = True
        self.get_logger().warn(reason)
        try:
            self.frame_source.land()
        except Exception as exc:
            self.get_logger().error(f"Tello land failed: {exc}")

    @staticmethod
    def clip_rc(value, limit):
        return int(max(-limit, min(limit, round(value))))

    def log_detection(self, state):
        now = time.time()
        if now - self.last_detection_log_time < 1.0:
            return
        if state == "tracking":
            self.get_logger().info("Face detected; publishing tracking command.")
        else:
            self.get_logger().warn("No face detected; publishing hover/search command.")
        self.last_detection_log_time = now

    def draw_debug(self, frame, face):
        if face is not None:
            x, y, w, h = face[:4]
            confidence = face[4] if len(face) > 4 else 1.0
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (x + w // 2, y + h // 2), 5, (0, 255, 255), cv2.FILLED)
            cv2.putText(
                frame,
                f"{self.detector.name} {confidence:.2f}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        cv2.line(frame, (self.frame_width // 2, 0), (self.frame_width // 2, self.frame_height), (255, 255, 0), 1)
        cv2.line(frame, (0, self.frame_height // 2), (self.frame_width, self.frame_height // 2), (255, 255, 0), 1)
        cv2.putText(
            frame,
            f"face tracking: {self.detector.name}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def shutdown(self):
        if self.frame_source is not None:
            self.frame_source.close()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
