import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus  # 导入自定义消息接口
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QVBoxLayout, 
                             QFrame, QSizePolicy)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
import sys
import time

# 主ROS节点（仅处理话题订阅，不操作GUI）
class DisplaySubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.get_logger().info(f"{node_name}, 启动！")
        # 初始化变量，避免AttributeError
        self.host_name = None
        self.data_str = "等待接收系统状态数据..."
        # 创建订阅器（注意：你原代码里变量名写错了，是subscription不是sublisher）
        self.status_subscription = self.create_subscription(
            SystemStatus,
            "sys_status",
            self.display_callback,
            10  # QoS深度
        )

    def display_callback(self, msg):
        # 仅处理数据格式化，不操作GUI
        self.host_name = msg.host_name
        self.data_str = f"""
---------------系统状态---------------
计算机时间:\t{msg.stamp.sec}s
主机名字:\t{msg.host_name}
CPU使用率:\t{msg.cpu_percent}%
内存使用率:\t{msg.memory_percent}%
内存总大小:\t{msg.memory_total}MB
剩余内存大小:\t{msg.memory_available}MB
网络发送数据总量:\t{msg.net_sent}MB
网络接收数据总量:\t{msg.net_recv}MB
        """
        # 触发信号（如果有GUI线程绑定）
        if hasattr(self, 'status_signal'):
            self.status_signal.emit(self.data_str)
# ROS 节点在 display_callback 中收到新数据，更新 self.data_str。
# 检查节点是否绑定了 status_signal（即是否有 GUI 在监听）。hasattr(检查自身是否具有这样的属性)
# 如果绑定了，就通过 emit() 把数据发送给 GUI，触发界面更新；如果没有绑定，就跳过这一步，避免报错。

# 定义ROS订阅线程（分离ROS逻辑和GUI逻辑，保证线程安全）
class ROSSubscriberThread(QThread):
    # 定义信号，用于向主线程传递数据（PyQt线程安全通信方式）
    status_signal = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.is_running = True

    def run(self):
        # 运行ROS的spin，持续订阅话题
        while self.is_running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)  # 非阻塞spin，避免卡住
            time.sleep(0.1)

    def stop(self):
        self.is_running = False
        self.quit()
        self.wait()

# 主GUI窗口类
class StatusDisplayWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()
        # 绑定ROS节点的信号到GUI更新函数
        self.node.status_signal = self.ros_sub_thread.status_signal
        self.ros_sub_thread.status_signal.connect(self.update_status_label)

    def init_ui(self):
        # 设置窗口基本属性
        self.setWindowTitle("系统状态监控")
        self.setFixedSize(500, 300)  # 固定窗口尺寸

        # 创建布局和标签
        self.layout = QVBoxLayout()
        self.status_label = QLabel()
        # 设置标签样式，提升显示效果
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.status_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.status_label.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.status_label.setStyleSheet("padding: 17px; font-size: 17px;")
        self.status_label.setText("等待接收系统状态数据...")

        # 添加组件到布局
        self.layout.addWidget(self.status_label)
        self.setLayout(self.layout)

        # 创建并启动ROS订阅线程
        self.ros_sub_thread = ROSSubscriberThread(self.node)
        self.ros_sub_thread.start()

    def update_status_label(self, data_str):
        # 主线程更新GUI，保证线程安全
        self.status_label.setText(data_str)

    def closeEvent(self, event):
        # 窗口关闭时清理资源
        self.ros_sub_thread.stop()
        rclpy.shutdown()
        event.accept()

def main():
    # 1. 初始化ROS 2
    rclpy.init(args=sys.argv)
    # 2. 创建ROS节点
    display_node = DisplaySubNode("display_sub")
    # 3. 初始化PyQt5应用（必须在主线程）
    app = QApplication(sys.argv)
    # 4. 创建GUI窗口
    window = StatusDisplayWindow(display_node)
    window.show()
    # 5. 运行PyQt5事件循环（主线程）
    app.exec_()

if __name__ == "__main__":
    main()