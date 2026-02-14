import rclpy
from rclpy.node import Node
from demo_python_pkg.class_study import PersonNode
class WriterNode(PersonNode):
    def __init__(self,node_name:str,name:str,age:int,book:str):
        print("WriterNode __init__ 方法被调用了,添加了一个属性")
        super().__init__(node_name,name,age)#调用父类的 __init__
        self.book = book

def main():
    rclpy.init()
    node = WriterNode("laowang","法外狂徒张三",18,"论快速入狱")
    node.eat("鱼香肉丝")
    rclpy.spin(node)
    rclpy.shutdown()