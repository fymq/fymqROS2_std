import rclpy
from  rclpy.node  import Node
from chapt4_interfaces.srv import FaceDetector
import face_recognition
import cv2
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
import os
import time

from ament_index_python.packages import get_package_share_directory# 获取功能包share目录绝对路径
class FaceDetectNode(Node):
    def __init__(self):
        super().__init__("face_detect_node")
        self.service_ = self.create_service(FaceDetector,"face_detect",self.detect_face_callback)
        self.bridge = CvBridge()
        self.declare_parameter("number_of_times_to_upsample",1)
        self.declare_parameter("model","hog")
        self.number_of_times_to_upsample = self.get_parameter("number_of_times_to_upsample").value
        self.default_image_path = get_package_share_directory("demo_python_service")+"/resource/face.jpg"
        self.model=self.get_parameter("model").value
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"人脸检测服务已经启动!")
        """
        设置自身节点参数的方法
        self.set_parameters([rclpy.Parameter("model",rclpy.Parameter.Type.STRING,"cnn")])
        """
    
    def parameter_callback(self,parameters):
        for parameter in parameters:
            self.get_logger().info(f"{parameter.name}->{parameter.value}")
            if parameter.name == "number_of_times_to_upsample":
                self.number_of_times_to_upsample = parameter.value
            if parameter.name == "model":
                self.model = parameter.value
        return SetParametersResult(successful=True)#必须！用于告诉ros2参数更新的成功与否
        


    def detect_face_callback(self,request,response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info(f"未检测到图像，使用默认图像")
        #cv_image已经是一个opencv的图像了
        #记录处理时间
        start_time = time.time()
        self.get_logger().info(f"加载完图像，开始识别")
        #检测人脸，返回位置
        face_locations = face_recognition.face_locations(cv_image,
        number_of_times_to_upsample = self.number_of_times_to_upsample,
        model = self.model)
        response.use_time = time.time() - start_time
        response.number = len(face_locations)
        #绘制人脸框
        for top,right,bottom,left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response #必须返回response
def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()
    