import launch
import launch_ros

# launch 的三大组件
# 动作:除了是一个节点，还可以是一句打印、一段终端指令、甚至是另外一个launch
# 替换:使用launch的参数替换节点的参数值
# 条件:利用条件可以决定哪些动作启动、哪些不启动。相当于if




def generate_launch_description():
    #1.声明一个参数
    action_declare_arg_background_g = launch.actions.DeclareLaunchArgument("launch_arg_bg",default_value = "150")
    #只有这样才可以在ros2 launch demo_python_service demo.launch.py + launch_arg_bg:=255
    
    "产生launch描述"
    action_node_turtlesim_node = launch_ros.actions.Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters = [{"background_g": launch.substitutions.LaunchConfiguration("launch_arg_bg",default = "150")}],
        #将上面的参数拿过来替换这个节点的参数，这里的默认default可以不写
        output = "screen"
    )
    
    action_node_patrol_control = launch_ros.actions.Node(
        package="demo_python_service",
        executable="turtle_patrol_control",
        output = "log"
    )

    action_node_patrol_client = launch_ros.actions.Node(
        package="demo_python_service",
        executable="turtle_patrol_client",
        output = "both"#log和screen
    )

    return launch.LaunchDescription([
        #action动作
        action_node_turtlesim_node,
        action_node_patrol_control,
        action_node_patrol_client
    ])