import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math

class TurtlePubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name},启动！")

        # 生成第一个随机目标点（仅返回x/y，简化数据）
        self.ta_pose = self.generate_random_target()
        self.get_logger().info(f"{node_name}启动!第一个随机目标点:x={self.ta_pose[0]:.2f}, y={self.ta_pose[1]:.2f}")
        
        # 到达目标点的阈值（距离小于0.1米则认为到达）
        self.arrive_threshold = 0.1
        # 创建订阅者（接收海龟位姿）
        self.turtle_subscripter = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        # 创建发布者（发送速度指令）
        self.turtle_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
    
    def generate_random_target(self):
        """生成随机目标点（仅返回x/y，简化冗余数据）"""
        random_x = random.uniform(1.0, 10.0)
        random_y = random.uniform(1.0, 10.0)
        return [random_x, random_y]  # 仅保留x/y，删除冗余的0
    
    def pose_callback(self, pos):
        # 优化日志输出：保留2位小数，提升可读性
        self.get_logger().info(
            f"当前位置x = {pos.x:.2f} y = {pos.y:.2f} 朝向角度:{pos.theta:.2f} 线速度：{pos.linear_velocity:.2f} 角速度:{pos.angular_velocity:.2f}"
        )
        
        # 计算到目标点的差值
        dx = self.ta_pose[0] - pos.x
        dy = self.ta_pose[1] - pos.y
        # 计算目标朝向
        target_theta = math.atan2(dy, dx)
        
        # 计算角度偏差并做最短路径修正（核心优化）
        theta_error = target_theta - pos.theta
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))
        
        # 计算到目标点的距离
        distance = math.hypot(dx, dy)
        
        # 判断是否到达目标点
        if distance < self.arrive_threshold:
            stop_msg = Twist()
            self.turtle_publisher.publish(stop_msg)
            self.get_logger().info("已到达目标点，停止移动！")
            # 生成新的随机目标点
            self.ta_pose = self.generate_random_target()
            self.get_logger().info(f"新随机目标点:x={self.ta_pose[0]:.2f}, y={self.ta_pose[1]:.2f}")
            return
        
        # 组装速度指令
        tw_msg = Twist()
        # 线速度：正比于距离，上限1.0 m/s（原逻辑保留，无问题）
        tw_msg.linear.x = min(distance * 0.7, 1.0)
        tw_msg.linear.y = 0.0
        tw_msg.linear.z = 0.0

        # 角速度：修正为双向限幅（-1.0 ~ 1.0），避免旋转过烈
        angular_vel = theta_error * 0.7
        tw_msg.angular.x = 0.0
        tw_msg.angular.y = 0.0
        tw_msg.angular.z = min(max(angular_vel, -1.0), 1.0)
        
        # 发布速度指令
        self.turtle_publisher.publish(tw_msg)

def main():
    rclpy.init()
    node = TurtlePubNode("turtle_pose")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()