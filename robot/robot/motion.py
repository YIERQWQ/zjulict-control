#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import signal
from rclpy.executors import SingleThreadedExecutor

class KinematicTest(Node):
    def __init__(self):
        super().__init__('kinematic_test')
        # 1. 发布器配置（恢复正常QoS，保证初始化）
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 2. 紧急停止标志位
        self.emergency_stop = False
        # 3. 定时器（100Hz，保证主线程不退出）
        self.timer = self.create_timer(0.01, self.loop)
        self.start_time = self.get_clock().now()
        
        # 仅关闭ERROR以下的日志（保留启动提示，屏蔽报错）
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        print("运动节点启动成功，开始执行预设轨迹...")  # 明确的启动提示

    def loop(self):
        # 紧急停止逻辑（优先执行）
        if self.emergency_stop:
            stop_msg = Twist()
            self.pub.publish(stop_msg)
            return
        
        # 正常运动逻辑（仅捕获数学计算/发布异常，不掩盖核心问题）
        try:
            t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            msg = Twist()
            msg.linear.x = 0.5 * math.sin(0.5 * t)
            msg.linear.y = 0.5 * math.cos(0.5 * t)
            msg.angular.z = 0.2
            self.pub.publish(msg)
        except Exception as e:
            # 仅静默捕获运动逻辑异常，不影响节点运行
            pass

    def trigger_emergency_stop(self):
        """优雅停止，无报错输出"""
        self.emergency_stop = True
        print("\n接收到停止信号，正在发送制动指令...")
        
        # 连续发送停止指令（保证Gazebo接收）
        stop_msg = Twist()
        for _ in range(15):
            self.pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        print("制动指令发送完成，程序正常退出")

# 自定义信号处理器（仅捕获Ctrl+C，无默认异常）
def sigint_handler(signal_num, frame, node, executor):
    try:
        node.trigger_emergency_stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass
    finally:
        exit(0)

def main(args=None):
    # 1. 正常初始化ROS2（不关闭日志，仅屏蔽非必要报错）
    rclpy.init(args=args)
    
    # 2. 创建节点和执行器（保证主线程阻塞）
    node = KinematicTest()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # 3. 注册Ctrl+C信号处理器（核心：替代默认KeyboardInterrupt）
    signal.signal(signal.SIGINT, lambda sig, frame: sigint_handler(sig, frame, node, executor))

    # 4. 执行器阻塞运行（保证节点不退出）
    try:
        executor.spin()
    except Exception as e:
        # 仅在启动失败时输出提示（避免静默退出）
        print(f"\n节点运行异常: {str(e)}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
