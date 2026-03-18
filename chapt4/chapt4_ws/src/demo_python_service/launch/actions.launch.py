import launch
import launch_ros
from ament_index_python import get_package_share_directory#获取包路径
# launch 的三大组件
# 动作:除了是一个节点，还可以是一句打印、一段终端指令、甚至是另外一个launch
# 替换:使用launch的参数替换节点的参数值
# 条件:利用条件可以决定哪些动作启动、哪些不启动。相当于if


def generate_launch_description():
    #1.声明一个参数
    action_declare_startup_rqt = launch.actions.DeclareLaunchArgument("startup_rqt",default_value = "False")
    
    startup_rqt = launch.substitutions.LaunchConfiguration("startup_rqt")
    
    #动作1：启动其他launch
    multisim_launch_path = [get_package_share_directory("turtlesim"),"/launch/","multisim.launch.py"]
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_path
        )
    )

    #动作2 打印数据
    action_log_info = launch.actions.LogInfo(msg = str(multisim_launch_path))

    #动作3 执行进程
    action_topic_list = launch.actions.ExecuteProcess(
        condition = launch.conditions.IfCondition(startup_rqt),#条件控制动作 类似 if(startup_rqt)
        cmd = ["rqt"]
        )

    #动作4 组织动作成组
    action_group = launch.actions.GroupAction([
        #动作5 定时启动动作
        launch.actions.TimerAction(period = 2.0,actions = [action_include_launch]),
        launch.actions.TimerAction(period = 4.0,actions = [action_topic_list]),
    ])

    return launch.LaunchDescription([
        #action动作
        # action_declare_startup_rqt,
        action_log_info,
        action_group,
    ])
# 六、运行效果
# 立刻打印：多海龟 Launch 路径
# 2 秒后：自动弹出 2 只海龟的仿真窗口
# 4 秒后：如果参数为true，自动打开 rqt 工具
