#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class ArucoController(Node):
    def __init__(self):
        super().__init__('aruco_controller')
        
        # 1. 订阅视觉节点发来的“误差坐标”
        # 我们约定：x是左右偏差，y是上下偏差，z是面积（代表距离）
        self.error_sub = self.create_subscription(
            Point,
            '/aruco/error',
            self.error_callback,
            10
        )
        
        # 2. 发布速度指令给无人机
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        
        # 3. 设定 PID 控制的比例系数 (可以根据仿真情况微调)
        self.kp_linear = 0.002   # 控制平移的速度系数
        self.kp_angular = 0.005  # 控制旋转的速度系数
        
        self.get_logger().info("🎯 控制决策节点已启动，等待视觉误差数据...")

    def error_callback(self, msg):
        cmd = Twist()
        
        # 如果 z (面积) 为 -1，说明视觉节点丢失了目标
        if msg.z == -1.0:
            self.get_logger().warn("目标丢失！悬停等待...")
            # 发送全 0 速度，原地悬停
            self.vel_pub.publish(cmd)
            return

        # --- 第一阶段：左右对齐 (Yaw 旋转) ---
        # msg.x 大于 0 说明码在画面右边，无人机需要向右转 (负的角速度)
        cmd.angular.z = -msg.x * self.kp_angular
        
        # --- 第二阶段：高度对齐 (Z轴上升/下降) ---
        # msg.y 大于 0 说明码在画面下方，无人机需要下降 (负的 Z 速度)
        cmd.linear.z = -msg.y * self.kp_linear
        
        # --- 第三阶段：前后距离对齐 (Y轴前进/后退) ---
        # 假设我们希望码的面积保持在 30000 像素左右
        target_area = 30000.0
        area_error = target_area - msg.z
        
        # 如果面积太小（离得远），就往前飞（X方向正速度，注意 Tello 坐标系 X 是向前）
        # 这里为了安全先不给太大的前进速度，你可以慢慢调大 0.00005 这个系数
        cmd.linear.x = area_error * 0.00001 
        
        # 发布最终混合好的速度指令
        self.vel_pub.publish(cmd)
        self.get_logger().info(f"追踪中 | 角速度: {cmd.angular.z:.2f}, 升降: {cmd.linear.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()