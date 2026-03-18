import rclpy
from  rclpy.node  import Node
from chapt4_interfaces.srv import Patrol
import os
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
import random

class PatrolClientNode(Node):
    def __init__(self):
        super().__init__("turtle_patrol_client")
        self.get_logger().info(f"海龟训练客户端已经启动!")
        self.client = self.create_client(Patrol,"patrol")
        self.create_timer(5,self.send_request)#定时器5秒一次

    def call_set_parameters(self,parameters):
        """
        调用服务,修改参数值
        """
        #1.创建一个客户端，等待服务上线
        update_param_client = self.create_client(SetParameters,"/turtle_pose/set_parameters")
        while update_param_client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f"等待参数更新服务上线中...")
        #2，创建request
        request = SetParameters.Request()
        request.parameters = parameters
        #3.调用服务端更新参数
        future = update_param_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)#等待服务端响应
        response = future.result()#获取响应
        return response
    
    def update_detect_k(self,k):
        #1.创建参数对象
        param = Parameter()
        param.name = "k"
        #2.赋值
        """
        Parameter[] parameters
        string name
        ParameterValue value 此处是一个新的对象(嵌套)
                uint8 type
                bool bool_value
                int64 integer_value
                float64 double_value
                string string_value
                byte[] byte_array_value
                bool[] bool_array_value
                int64[] integer_array_value
                float64[] double_array_value
                string[] string_array_value

-------------------------
        # Indicates whether setting each parameter succeeded or not and why.
        SetParametersResult[] results
                bool successful
                string reason
        """
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_DOUBLE#将数字定义了别名，增强可读性(防止单写数字认不出来)
        param_value.double_value = k
        param.value = param_value
        #3.请求更新参数
        response = self.call_set_parameters([param])
        for result in response.results:
            self.get_logger().info(f"设置参数结果:{result.successful}{result.reason}")
        
    def send_request(self):
        #判断服务端是否在线
        while self.client.wait_for_service(timeout_sec = 1.0) is False:
            self.get_logger().info("等待服务端上线")
        #已有服务端
        #构建请求体
        request = Patrol.Request()
        request.target_x = round(random.uniform(1,10),2)
        request.target_y = round(random.uniform(1,10),2)
        self.get_logger().info(f"海龟训练客户端新的目标点是{request.target_x},{request.target_y}")
        #发送请求并等待处理完成
        #发送异步请求，注册回调函数（不阻塞）
        future = self.client.call_async(request)
        #用lambda传递future,在响应到达时自动调用show_response
        future.add_done_callback(lambda future: self.show_response(future.result()))

    #客户端接受服务返回的数据
    def show_response(self,response):
        self.get_logger().info(f"客户端接收的返回数据{response}")

def main():
    rclpy.init()
    node = PatrolClientNode()
    node.update_detect_k(1.5)
    # node.send_request()
    # node.update_detect_k(2.5)
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()
