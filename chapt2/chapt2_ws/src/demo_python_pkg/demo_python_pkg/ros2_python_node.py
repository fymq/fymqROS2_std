import rclpy
from rclpy.node import Node

def main():
    rclpy.init()#初始化工作，分配资源
    node = Node("python_node")
    node.get_logger().info("你好python Node!")
    node.get_logger().warn("你好python Node!")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()