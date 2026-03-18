import rclpy
from  rclpy.node  import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from cv_bridge import CvBridge
import os
import time
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType

from ament_index_python.packages import get_package_share_directory# 获取功能包share目录绝对路径
class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__("face_detect_client_node")
        self.default_image_path = get_package_share_directory("demo_python_service")+"/resource/face2.jpg"
        self.bridge = CvBridge()
        self.get_logger().info(f"人脸检测客户端已经启动!")
        self.client = self.create_client(FaceDetector,"face_detect")
        self.image = cv2.imread(self.default_image_path)

    def call_set_parameters(self,parameters):
        """
        调用服务,修改参数值
        """
        #1.创建一个客户端，等待服务上线
        update_param_client = self.create_client(SetParameters,"/face_detect_node/set_parameters")
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
    
    def update_detect_model(self,model):
        """
        根据传入的model,构造parameters,然后调用call_set_parameters更新服务端参数
        """
        #1.创建参数对象
        param = Parameter()
        param.name = "model"
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
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING#将数字定义了别名，增强可读性(防止单写数字认不出来)
        param.value = param_value
        #3.请求更新参数
        response = self.call_set_parameters([param])
        for result in response.results:
            self.get_logger().info(f"设置参数结果:{result.successful}{result.reason}")

    def send_request(self):
        #1.判断服务端是否上线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f"等待服务上线中...")
        #2.构造request对象
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        #3.发送请求等待处理完成
        future = self.client.call_async(request)#现在future并没有响应结果，
        #需要等待服务端处理完成才会将结果放进future中
        # while not future.done():
        #     time.sleep(1.0)#休眠当前线程,等待服务完成,单线程会造成无法接受服务端的响应返回
        rclpy.spin_until_future_complete(self,future)#等待服务端响应
        response = future.result()#获取响应
        self.get_logger().info(f"接收到响应，共有{response.number}张人脸,耗时{response.use_time}秒")
        #self.show_response(response)
        #要正常显示需要把上面代码的注释删除(因为展示窗口会堵塞线程)

    def show_response(self,response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image,(left,top),(right,bottom),(255,0,0),4)#图像 位置 位置 颜色 宽度
        cv2.imshow("Face Detecte Result ",self.image)
        cv2.waitKey(0)#阻塞，会导致spin无法运行

def main():
    rclpy.init()
    node = FaceDetectClientNode()
#此处为发送请求服务
    node.update_detect_model("cnn")
    node.send_request()
    node.update_detect_model("hog")
    node.send_request()

    rclpy.spin(node)
    rclpy.shutdown()
        
