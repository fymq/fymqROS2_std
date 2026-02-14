import espeakng
import threading
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import time

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name},启动！")
        self.novel_queue = Queue()
        self.novel_sub = self.create_subscription(String,"novel",self.novel_callback,10)#创建订阅者
        self.speech_thread = threading.Thread(target = self.speaker_thread)#创建线程函数
        self.speech_thread.start()#开启线程

    def novel_callback(self,msg):
        self.novel_queue.put(msg.data)#放队列
        
    def speaker_thread(self):
        speaker = espeakng.Speaker()#创建读者对象
        speaker.voice = "zh"

        while rclpy.ok():#检测当前ROS上下文是否ok
            if self.novel_queue.qsize()> 0:
                text = self.novel_queue.get()
                self.get_logger().info(f"正在朗读：{text}")#打印朗读文本
                speaker.say(text)#朗读文本
                speaker.wait()#等待朗读完毕
            else:
                #休眠当前线程1s
                time.sleep(1)

def main():
    rclpy.init()
    node = NovelSubNode("novel_sub")
    rclpy.spin(node)
    rclpy.shutdown()