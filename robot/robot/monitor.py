#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys
import signal  # 引入信号处理模块
# 新增：配置日志器屏蔽ROS2输出
import logging

# 全局屏蔽ROS2的日志输出（兼容旧版本）
logging.getLogger('rclpy').setLevel(logging.CRITICAL)
logging.getLogger('rosout').setLevel(logging.CRITICAL)

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('robot_monitor')

        # 关闭节点所有日志输出（兼容旧版本写法）
        try:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        except:
            # 兼容更旧版本的ROS2
            pass

        # 1. 订阅 IMU
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        
        # 2. 订阅指令速度
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # 3. 订阅真值里程计
        self.odom_sub = self.create_subscription(Odometry, '/odom_ground_truth', self.odom_callback, 10)

        # 数据缓存
        self.latest_imu = None
        self.latest_cmd = None
        self.latest_odom = None

        # 路程积分相关
        self.total_distance = 0.0
        self.last_pos = None  # (x, y)

        # 定时刷新显示 (20Hz)
        self.timer = self.create_timer(0.05, self.print_dashboard)

    def imu_callback(self, msg):
        self.latest_imu = msg

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def odom_callback(self, msg):
        # 缓存最新里程计数据
        self.latest_odom = msg
        
        # 路程积分计算 (基于位置变化)
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        if self.last_pos is not None:
            dx = current_x - self.last_pos[0]
            dy = current_y - self.last_pos[1]
            dist_step = math.sqrt(dx**2 + dy**2)
            self.total_distance += dist_step
        
        self.last_pos = (current_x, current_y)

    def print_dashboard(self):
        # 清屏
        sys.stdout.write("\033[H\033[J")
        print("=" * 30)
        # --- 板块 1: 指令速度 (输入) ---
        print(f"【指令速度 (Command)】(来自 /cmd_vel)")
        if self.latest_cmd:
            print(f"  Vx : {self.latest_cmd.linear.x: .4f} m/s")
            print(f"  Vy : {self.latest_cmd.linear.y: .4f} m/s")
            print(f"  Wz : {self.latest_cmd.angular.z: .4f} rad/s")
        else:
            print("  [等待指令...]")
        print("-" * 30)

        # --- 板块 2: 真实速度 (输出) ---
        print(f"【真实速度 (Ground Truth)】(来自 Gazebo Odom Twist)")
        if self.latest_odom:
            vel = self.latest_odom.twist.twist.linear
            ang = self.latest_odom.twist.twist.angular
            print(f"  Vx : {vel.x: .4f} m/s")
            print(f"  Vy : {vel.y: .4f} m/s")
            print(f"  Wz : {ang.z: .4f} rad/s")
        else:
            print("  [等待里程计数据...]")
        print("-" * 30)

        # --- 板块 3: 绝对位置与路程 ---
        print(f"【绝对位置与路程】(来自 Gazebo Odom Twist)")
        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            ori = self.latest_odom.pose.pose.orientation
            _, _, yaw = self.euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
            print(f"  X : {pos.x: .4f} m")
            print(f"  Y : {pos.y: .4f} m")
            print(f"  Yaw: {math.degrees(yaw): .2f}°")
            print(f"  🚩 总路程: {self.total_distance: .4f} m")
        else:
            print("  [等待里程计数据...]")
        print("-" * 30)

        # --- 板块 4: IMU 数据 ---
        print(f"【IMU 传感器数据】")
        if self.latest_imu:
            # 解析 IMU Yaw
            iq = self.latest_imu.orientation
            _, _, imu_yaw = self.euler_from_quaternion(iq.x, iq.y, iq.z, iq.w)
            
            print(f"  [姿态] IMU Yaw: {math.degrees(imu_yaw): .2f}°")
            print(f"  [角速度] Wz: {self.latest_imu.angular_velocity.z: .4f} rad/s")
            print(f"  [加速度] Ax: {self.latest_imu.linear_acceleration.x: .4f} m/s²")
            print(f"  [加速度] Ay: {self.latest_imu.linear_acceleration.y: .4f} m/s²")
        else:
            print("  [等待IMU数据...]")
            
        print("="*30)

    def euler_from_quaternion(self, x, y, z, w):
        """标准四元数转欧拉角算法"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

# 自定义SIGINT处理器（静默处理Ctrl+C，无任何输出）
def sigint_handler(signal_num, frame, node):
    try:
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass
    finally:
        exit(0)

def main(args=None):
    # 修复：移除不兼容的 logging_enabled 参数
    rclpy.init(args=args)
    
    node = RobotMonitor()
    
    # 注册Ctrl+C信号处理器（替代默认的KeyboardInterrupt）
    signal.signal(signal.SIGINT, lambda sig, frame: sigint_handler(sig, frame, node))
    
    # 静默运行节点，捕获所有异常但无输出
    try:
        rclpy.spin(node)
    except:
        pass

if __name__ == '__main__':
    main()
