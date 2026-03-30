import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener,Buffer #动态坐标监听器
from geometry_msgs.msg import TransformStamped #消息接口
from tf_transformations import euler_from_quaternion #欧拉角转四元数函数
import math #角度转弧度函数

class TFListenerNode(Node):
    def __init__(self):
        super().__init__("tf_listener")
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_,self)
        self.timer_ = self.create_timer(1.0, self.get_transform)#每1秒监听一次TF

    def get_transform(self):
        """
        实时查询坐标关系 buffer_
        """
        try:
            result = self.buffer_.lookup_transform("base_link","bottle_link",rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f"平移:{transform.translation.x},{transform.translation.y},{transform.translation.z}")
            self.get_logger().info(f"四元数:{transform.rotation.x},{transform.rotation.y},{transform.rotation.z},{transform.rotation.w}")
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w])
            self.get_logger().info(f"旋转:{rotation_euler}")
        except Exception as e:
            self.get_logger().warn(f"查询坐标关系失败:原因{str(e)}")

def main():
    rclpy.init()
    node = TFListenerNode() #打印监听到的坐标关系，              
    rclpy.spin(node)
    rclpy.shutdown()
