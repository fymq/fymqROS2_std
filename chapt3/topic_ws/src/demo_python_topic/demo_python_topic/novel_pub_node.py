import rclpy
from rclpy.node import Node
import requests#用于获取网址内容
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name},启动！")
        self.novels_queue = Queue()#创建队列
        self.novel_publisher = self.create_publisher(String,"novel",10)#发布者
        self.create_timer(5,self.timer_callback)#时间周期
        

    def timer_callback(self):#回调函数
        if self.novels_queue.qsize()>0:
            line = self.novels_queue.get()

            msg = String()#发布操作
            msg.data = line
            self.novel_publisher.publish(msg)
            self.get_logger().info(f"发布了：{msg}")

    def download(self,url):
        resqonse = requests.get(url)
        resqonse.encoding = "utf-8"
        text = resqonse.text
        self.get_logger().info(f"{url},{len(text)}！")
        for line in text.splitlines():#分割数据
            self.novels_queue.put(line)
        #self.novel_publisher.publish()

def main():
    rclpy.init()
    node = NovelPubNode("novel_pub")
    node.download("http://0.0.0.0:8000/src/demo_python_topic/resource/novel1.txt")
    rclpy.spin(node)
    rclpy.shutdown()