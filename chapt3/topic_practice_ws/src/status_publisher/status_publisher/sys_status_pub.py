import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus#导入消息接口
import psutil
import platform

class SysStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name},启动！")
        self.status_publisher = self.create_publisher(SystemStatus,"sys_status",10)
        self.timer = self.create_timer(1,self.timer_callback)
    #builtin_interfaces/Time stamp #记录时间戳
    #         int32 sec
    #         uint32 nanosec
    # string host_name #主机名字
    # float32 cpu_percent #CPU使用率
    # float32 memory_percent #内存使用率
    # float32 memory_total #内存总大小
    # float32 memory_available #剩余内存大小
    # float64 net_sent #网络发送数据总量
    # float64 net_recv #网络接收数据总量
    def timer_callback(self):
        cpu_percent = psutil.cpu_percent()
        memory_info = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()

        msg = SystemStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_percent = memory_info.percent
        msg.memory_total = memory_info.total / 1024 / 1024
        msg.memory_available = memory_info.available / 1024 / 1024
        msg.net_sent =  net_io_counters.bytes_sent/ 1024 / 1024
        msg.net_recv=  net_io_counters.bytes_recv/ 1024 / 1024

        self.get_logger().info(f"发布:{str(msg)}")
        self.status_publisher.publish(msg)


def main():
    rclpy.init()
    node = SysStatusPub("sys_status_pub")
    rclpy.spin(node)
    rclpy.shutdown()