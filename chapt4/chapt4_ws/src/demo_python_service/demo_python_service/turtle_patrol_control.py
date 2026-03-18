import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from chapt4_interfaces.srv import Patrol
from rcl_interfaces.msg import SetParametersResult

class TurtleControlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name},启动！")
        #初始移动目标点
        self.targetPos_x = 1.0
        self.targetPos_y = 1.0
        #将以下两个值参数化
        self.declare_parameter("k",0.7)
        self.declare_parameter("max_speed",1.0)
        #比例系数k(关乎线速度与角速度)
        self.k = self.get_parameter("k").value
        #最大线速度
        self.max_speed = self.get_parameter("max_speed").value
        # 到达目标点的阈值（距离小于0.1米则认为到达）
        self.arrive_threshold = 0.1
        self.services_ = self.create_service(Patrol,"patrol",self.Turtle_service_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"海龟巡逻服务已启动！")
        # 创建订阅者（接收海龟位姿）
        self.turtle_subscripter = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        # 创建发布者（发送速度指令）
        self.turtle_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    #参数回调函数
    def parameter_callback(self,parameters):
        for parameter in parameters:
            self.get_logger().info(f"{parameter.name}->{parameter.value}")
            if parameter.name == "k":
                self.k = parameter.value
            if parameter.name == "max_speed":
                self.max_speed = parameter.value
        return SetParametersResult(successful=True)#必须！用于告诉ros2参数更新的成功与否
    
    def pose_callback(self, pos):
        # 优化日志输出：保留2位小数，提升可读性
        self.get_logger().info(
            f"当前位置x = {pos.x:.2f} y = {pos.y:.2f} 朝向角度:{pos.theta:.2f} 线速度：{pos.linear_velocity:.2f} 角速度:{pos.angular_velocity:.2f}"
        )
        
        # 计算到目标点的差值
        dx = self.targetPos_x - pos.x
        dy = self.targetPos_y - pos.y
        # 计算目标朝向
        target_theta = math.atan2(dy, dx)
        
        # 计算角度偏差并做最短路径修正（核心优化）
        theta_error = target_theta - pos.theta
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))
        
        # 计算到目标点的距离
        distance = math.hypot(dx, dy)
        
        # 判断是否到达目标点
        if distance < self.arrive_threshold:
            stop_msg = Twist()
            self.turtle_publisher.publish(stop_msg)
            self.get_logger().info("已到达目标点，请选择新的目标点！")
        
        # 组装速度指令
        tw_msg = Twist()
        # 线速度：正比于距离，上限1.0 m/s（原逻辑保留，无问题）
        tw_msg.linear.x = min(distance * self.k, self.max_speed)
        tw_msg.linear.y = 0.0
        tw_msg.linear.z = 0.0

        # 角速度：修正为双向限幅（-1.0 ~ 1.0），避免旋转过烈
        angular_vel = theta_error * self.k
        tw_msg.angular.x = 0.0
        tw_msg.angular.y = 0.0
        tw_msg.angular.z = min(max(angular_vel, -1.0), 1.0)
        
        # 发布速度指令
        self.turtle_publisher.publish(tw_msg)
    def Turtle_service_callback(self,request,response):
        self.targetPos_x = request.target_x
        self.targetPos_y = request.target_y
        response.result = response.SUCCESS
        return response

def main():
    rclpy.init()
    node = TurtleControlNode("turtle_pose")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()