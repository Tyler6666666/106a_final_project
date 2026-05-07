#!/usr/bin/env python3
import time

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from tello_msgs.srv import TelloAction


class FaceController(Node):
    def __init__(self):
        super().__init__("face_controller_node")

        self.declare_parameter("face_pose_topic", "/face/pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tello_action_service", "/tello_action")
        self.declare_parameter("auto_takeoff", True)
        self.declare_parameter("require_face_before_takeoff", True)
        self.declare_parameter("face_confirm_sec", 2.0)
        self.declare_parameter("follow_after_takeoff_sec", 2.0)
        self.declare_parameter("land_on_face_loss", True)
        self.declare_parameter("face_loss_land_sec", 5.0)
        self.declare_parameter("target_face_area_ratio", 0.10)
        self.declare_parameter("forward_area_threshold", 0.075)
        self.declare_parameter("min_forward_speed", 0.08)
        self.declare_parameter("lost_timeout_sec", 0.6)
        self.declare_parameter("deadband_x", 0.08)
        self.declare_parameter("deadband_y", 0.08)
        self.declare_parameter("deadband_area", 0.025)
        self.declare_parameter("pose_filter_alpha", 0.35)
        self.declare_parameter("max_forward_speed", 0.18)
        self.declare_parameter("max_vertical_speed", 0.15)
        self.declare_parameter("max_yaw_speed", 0.35)
        self.declare_parameter("kp_forward", 0.85)
        self.declare_parameter("kp_vertical", 0.45)
        self.declare_parameter("kp_yaw", 0.70)
        self.declare_parameter("kd_yaw", 0.08)
        self.declare_parameter("kd_forward", 0.04)
        self.declare_parameter("search_when_lost", False)
        self.declare_parameter("search_yaw_speed", 0.18)

        face_pose_topic = self.get_parameter("face_pose_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        tello_action_service = self.get_parameter("tello_action_service").value
        self.auto_takeoff_enabled = bool(self.get_parameter("auto_takeoff").value)
        self.require_face_before_takeoff = bool(self.get_parameter("require_face_before_takeoff").value)
        self.face_confirm_sec = float(self.get_parameter("face_confirm_sec").value)
        self.follow_after_takeoff_sec = float(self.get_parameter("follow_after_takeoff_sec").value)
        self.land_on_face_loss = bool(self.get_parameter("land_on_face_loss").value)
        self.face_loss_land_sec = float(self.get_parameter("face_loss_land_sec").value)
        self.target_face_area_ratio = float(self.get_parameter("target_face_area_ratio").value)
        self.forward_area_threshold = float(self.get_parameter("forward_area_threshold").value)
        self.min_forward_speed = float(self.get_parameter("min_forward_speed").value)
        self.lost_timeout_sec = float(self.get_parameter("lost_timeout_sec").value)
        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)
        self.deadband_area = float(self.get_parameter("deadband_area").value)
        self.pose_filter_alpha = float(self.get_parameter("pose_filter_alpha").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_vertical_speed = float(self.get_parameter("max_vertical_speed").value)
        self.max_yaw_speed = float(self.get_parameter("max_yaw_speed").value)
        self.kp_forward = float(self.get_parameter("kp_forward").value)
        self.kp_vertical = float(self.get_parameter("kp_vertical").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.kd_yaw = float(self.get_parameter("kd_yaw").value)
        self.kd_forward = float(self.get_parameter("kd_forward").value)
        self.search_when_lost = bool(self.get_parameter("search_when_lost").value)
        self.search_yaw_speed = float(self.get_parameter("search_yaw_speed").value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.face_sub = self.create_subscription(Pose, face_pose_topic, self.face_cb, 10)
        self.action_client = self.create_client(TelloAction, tello_action_service)

        self.start_time = time.time()
        self.latest_pose = None
        self.smoothed_pose = None
        self.last_seen = 0.0
        self.prev_face_time = None
        self.confirmed_face_time = 0.0
        self.waiting_for_face = True
        self.last_log_time = 0.0
        self.last_takeoff_status_log = 0.0
        self.has_sent_takeoff = False
        self.takeoff_sent_time = None
        self.follow_mode_started = False
        self.follow_mode_logged = False
        self.has_sent_land = False
        self.prev_control_time = None
        self.prev_err_x = 0.0
        self.prev_err_area = 0.0
        self.filtered_dx = 0.0
        self.filtered_darea = 0.0

        self.create_timer(1.0 / 30.0, self.control_loop)
        self.create_timer(0.5, self.auto_takeoff)
        self.get_logger().info(f"Face controller listening on {face_pose_topic}; publishing {cmd_vel_topic}.")

    @staticmethod
    def clip(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def apply_deadband(value, deadband):
        return 0.0 if abs(value) < deadband else value

    def face_cb(self, msg):
        now = time.time()
        if self.prev_face_time is None or now - self.prev_face_time > 1.5:
            self.confirmed_face_time = 0.0
        else:
            self.confirmed_face_time += min(now - self.prev_face_time, 0.25)
        self.prev_face_time = now

        self.latest_pose = msg
        self.update_smoothed_pose(msg)
        self.last_seen = now
        self.waiting_for_face = False

    def update_smoothed_pose(self, msg):
        if self.smoothed_pose is None:
            self.smoothed_pose = Pose()
            self.smoothed_pose.position.x = msg.position.x
            self.smoothed_pose.position.y = msg.position.y
            self.smoothed_pose.position.z = msg.position.z
            self.smoothed_pose.orientation.z = msg.orientation.z
            return

        alpha = max(0.0, min(1.0, self.pose_filter_alpha))
        self.smoothed_pose.position.x = alpha * msg.position.x + (1.0 - alpha) * self.smoothed_pose.position.x
        self.smoothed_pose.position.y = alpha * msg.position.y + (1.0 - alpha) * self.smoothed_pose.position.y
        self.smoothed_pose.position.z = alpha * msg.position.z + (1.0 - alpha) * self.smoothed_pose.position.z
        self.smoothed_pose.orientation.z = alpha * msg.orientation.z + (1.0 - alpha) * self.smoothed_pose.orientation.z

    def control_loop(self):
        now = time.time()
        cmd = Twist()

        if self.has_sent_land:
            self.cmd_pub.publish(cmd)
            return

        if self.auto_takeoff_enabled and not self.has_sent_takeoff:
            self.cmd_pub.publish(cmd)
            return

        if self.has_sent_takeoff and self.takeoff_sent_time is not None:
            elapsed = now - self.takeoff_sent_time
            if elapsed < self.follow_after_takeoff_sec:
                self.cmd_pub.publish(cmd)
                return
            if not self.follow_mode_started:
                self.follow_mode_started = True
                if not self.follow_mode_logged:
                    self.get_logger().warn("Entering face follow mode.")
                    self.follow_mode_logged = True

        if self.latest_pose is None or now - self.last_seen > self.lost_timeout_sec:
            if self.should_land_on_face_loss(now):
                self.cmd_pub.publish(cmd)
                self.send_land_once("Face lost; landing.")
                return
            if self.search_when_lost and self.latest_pose is not None:
                cmd.angular.z = self.search_yaw_speed
            self.cmd_pub.publish(cmd)
            if not self.waiting_for_face:
                self.get_logger().warn("Face lost; hovering until face returns.")
                self.waiting_for_face = True
            return

        pose = self.smoothed_pose if self.smoothed_pose is not None else self.latest_pose
        err_x = self.apply_deadband(pose.position.x, self.deadband_x)
        err_y = self.apply_deadband(pose.position.y, self.deadband_y)
        face_area = pose.position.z
        err_area = self.apply_deadband(self.target_face_area_ratio - face_area, self.deadband_area)
        dx, darea = self.update_error_derivatives(err_x, err_area)

        cmd.angular.z = self.clip((-self.kp_yaw * err_x) - (self.kd_yaw * dx), self.max_yaw_speed)
        cmd.linear.z = self.clip(-self.kp_vertical * err_y, self.max_vertical_speed)
        cmd.linear.x = self.make_forward_speed(face_area, err_area, darea)
        self.cmd_pub.publish(cmd)

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"Face tracking: x={pose.position.x:.2f} "
                f"y={pose.position.y:.2f} area={face_area:.3f} "
                f"cmd=({cmd.linear.x:.2f}, {cmd.linear.z:.2f}, {cmd.angular.z:.2f})"
            )
            self.last_log_time = now

    def update_error_derivatives(self, err_x, err_area):
        now = time.time()
        if self.prev_control_time is None:
            self.prev_control_time = now
            self.prev_err_x = err_x
            self.prev_err_area = err_area
            return 0.0, 0.0

        dt = max(now - self.prev_control_time, 1e-3)
        raw_dx = (err_x - self.prev_err_x) / dt
        raw_darea = (err_area - self.prev_err_area) / dt
        self.filtered_dx = 0.35 * raw_dx + 0.65 * self.filtered_dx
        self.filtered_darea = 0.35 * raw_darea + 0.65 * self.filtered_darea

        self.prev_control_time = now
        self.prev_err_x = err_x
        self.prev_err_area = err_area
        return self.filtered_dx, self.filtered_darea

    def make_forward_speed(self, face_area, err_area, darea):
        if face_area < self.forward_area_threshold:
            forward_speed = max(self.min_forward_speed, self.kp_forward * (self.target_face_area_ratio - face_area))
            return self.clip(forward_speed, self.max_forward_speed)
        return self.clip((self.kp_forward * err_area) + (self.kd_forward * darea), self.max_forward_speed)

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
        if not self.action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /tello_action service before takeoff.")
            return

        req = TelloAction.Request()
        req.cmd = "takeoff"
        self.action_client.call_async(req)
        self.has_sent_takeoff = True
        self.takeoff_sent_time = time.time()
        self.get_logger().warn(
            f"Takeoff requested; entering face follow mode after {self.follow_after_takeoff_sec:.1f}s."
        )

    def should_land_on_face_loss(self, now):
        if not self.land_on_face_loss or self.has_sent_land:
            return False
        if self.auto_takeoff_enabled and not self.follow_mode_started:
            return False
        if self.latest_pose is None:
            return False
        return now - self.last_seen >= self.face_loss_land_sec

    def send_land_once(self, reason):
        if self.has_sent_land:
            return
        self.has_sent_land = True
        self.get_logger().warn(reason)
        if not self.action_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error("Cannot land: /tello_action service unavailable.")
            return
        req = TelloAction.Request()
        req.cmd = "land"
        self.action_client.call_async(req)

    def log_takeoff_status(self, message):
        now = time.time()
        if now - self.last_takeoff_status_log >= 1.0:
            self.get_logger().warn(message)
            self.last_takeoff_status_log = now

    def stop_motion(self):
        if rclpy.ok():
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = FaceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_motion()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
