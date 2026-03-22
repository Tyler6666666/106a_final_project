#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tello_msgs.srv import TelloAction
import time
import numpy as np

class SmartTracker(Node):
    def __init__(self):
        super().__init__('aruco_controller_node')
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/aruco/pose_3d', self.pose_cb, 10)
        self.takeoff_client = self.create_client(TelloAction, '/drone1/tello_action')

        # --- PD 控制参数 ---
        self.Kp_side = 0.60
        self.Kp_yaw = 2.80
        self.Kp_fwd = 0.40
        self.target_dist = 0.8 

        # --- 搜索逻辑参数 ---
        self.last_seen = 0.0
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.4      # 慢速旋转速度 (rad/s)
        self.search_duration = 15.0  # 转一圈大概需要的时间 (2*pi / 0.4 ≈ 15.7s)
        self.search_cooldown = False # 防止搜索完后立即重复搜索

        self.latest_pose = None
        self.start_time = time.time()
        self.has_sent_takeoff = False

        # 20Hz 高频控制
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

    def pose_cb(self, msg):
        self.latest_pose = msg
        self.last_seen = time.time()
        # 只要看到码，就重置搜索状态
        self.is_searching = False
        self.search_cooldown = False

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        # 基础保护：起飞前5秒不动
        if now - self.start_time < 5.0:
            return

        # --- 逻辑核心：状态判定 ---
        time_since_last_seen = now - self.last_seen

        # 1. 追踪模式：如果目标在视野内（0.6秒内看过）
        if self.latest_pose is not None and time_since_last_seen < 0.6:
            self.run_pd_control(cmd)
            self.is_searching = False

        # 2. 搜索模式：如果目标丢失超过 1.0 秒，且还没完成这一轮搜索
        elif time_since_last_seen > 1.0 and not self.search_cooldown:
            if not self.is_searching:
                self.get_logger().warn("目标丢失！启动 360 度慢速搜索...")
                self.is_searching = True
                self.search_start_time = now
            
            elapsed_search = now - self.search_start_time
            
            if elapsed_search < self.search_duration:
                # 执行慢速自转
                cmd.angular.z = self.search_speed
            else:
                # 已经转完一圈还没找到，停止搜索，进入冷却防止无限旋转
                self.get_logger().error("搜索完毕，未发现目标。进入挂起状态。")
                cmd.angular.z = 0.0
                self.is_searching = False
                self.search_cooldown = True # 必须重新看到码才能触发下次搜索
        
        # 3. 停止模式：既没看到码，也搜索结束了
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)

    def run_pd_control(self, cmd):
        """原有的高性能 PD 控制逻辑"""
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_pose.orientation.z 
        err_fwd = self.latest_pose.position.z - self.target_dist

        # 耦合转向逻辑
        v_yaw = (err_yaw * self.Kp_yaw) + (err_x * 2.0) 
        
        # 基础速度
        vy = err_x * self.Kp_side
        vx = err_fwd * self.Kp_fwd
        vz = -self.latest_pose.position.y * 1.2

        # 突破死区 (Boost)
        def boost(val, min_s=0.25):
            if abs(val) > 0.03:
                return val if abs(val) > min_s else (min_s if val > 0 else -min_s)
            return 0.0

        cmd.angular.z = float(np.clip(boost(v_yaw, 0.3), -1.0, 1.0))
        cmd.linear.x = float(np.clip(boost(vx, 0.2), -0.4, 0.4))
        cmd.linear.y = float(np.clip(boost(vy, 0.2), -0.4, 0.4))
        cmd.linear.z = float(np.clip(vz, -0.3, 0.3))

    def auto_takeoff(self):
        if not self.has_sent_takeoff and (time.time() - self.start_time > 4.0):
            if self.takeoff_client.wait_for_service(timeout_sec=1.0):
                req = TelloAction.Request()
                req.cmd = 'takeoff'
                self.takeoff_client.call_async(req)
                self.has_sent_takeoff = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SmartTracker())
    rclpy.shutdown()