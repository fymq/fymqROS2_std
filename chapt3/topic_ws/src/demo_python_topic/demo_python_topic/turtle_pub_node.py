import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtlePubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name},启动！")
        self.create_timer(1,self.timer_callback)
        self.turtle_publisher = self.create_publisher(Twist,"/turtle1/cmd_vel",10)#发布者
        

    def timer_callback(self):
        twist = Twist()#组装数据
        twist.angular.z = 0.5
        twist.linear.x  = 1.0
        self.turtle_publisher.publish(twist)
        self.get_logger().info(f"发布速度指令：前进{twist.linear.x} m/s,旋转{twist.angular.z} rad/s")

def main():
    rclpy.init()
    node = TurtlePubNode("turtle_publisher")
    rclpy.spin(node)
    rclpy.shutdown()