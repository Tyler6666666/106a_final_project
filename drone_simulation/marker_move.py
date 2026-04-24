#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class RealDroneSimMarker(Node):
    def __init__(self):
        super().__init__('marker_drone_sim')
        self.publisher_ = self.create_publisher(Twist, '/target_marker/cmd_vel', 10)

        # --- 飞行轨迹参数 ---
        self.A = 2.5        # X轴（长边）振幅 (米)
        self.B = 1.2        # Y轴（短边）振幅 (米)
        self.C = 0.8        # Z轴振幅 (米)
        self.W = 0.35       # 频率 (数值越大飞得越快)

        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Marker 模拟飞行已启动！正在执行空间 8 字航线...")

    def timer_callback(self):
        t = time.time() - self.start_time
        msg = Twist()

        # 1. 空间 8 字轨迹
        # x = A * sin(W*t)
        # y = B * sin(2*W*t)
        # z = C * cos(W*t)
        # 这样在穿过中点时高度也会同步变化，形成空间八字而不是平面八字加轻微起伏
        vx = self.A * self.W * math.cos(self.W * t)
        vy = self.B * 2 * self.W * math.cos(2 * self.W * t)
        vz = -self.C * self.W * math.sin(self.W * t)

        # 2. 计算偏航角速度 (Yaw Rate)
        # 让 Marker 的“脸”始终朝着它飞行的方向
        # 我们计算当前速度矢量的角度变化
        heading = math.atan2(vy, vx)
        if hasattr(self, 'last_heading'):
            # 计算角度差并转为角速度
            yaw_error = heading - self.last_heading
            # 处理 pi 到 -pi 的跳变
            if yaw_error > math.pi: yaw_error -= 2*math.pi
            if yaw_error < -math.pi: yaw_error += 2*math.pi
            msg.angular.z = yaw_error / 0.05 # 20Hz 频率

        self.last_heading = heading

        # 3. 填充指令
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealDroneSimMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publisher_.publish(Twist()) # 停止运动
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
