import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument  # 导入“声明启动参数”的动作。
from launch.conditions import IfCondition, UnlessCondition  # 导入根据参数决定是否启动节点的条件。
from launch.substitutions import Command, LaunchConfiguration  # 导入命令替换和启动参数读取工具。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。
from launch_ros.parameter_descriptions import ParameterValue  # 导入 ROS 2 参数值描述对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    package_share = get_package_share_directory("mysystem")  # 获取 mysystem 软件包的 share 目录。
    urdf_file = os.path.join(package_share, "urdf", "robot.urdf")  # 拼出机器人 URDF 文件的完整路径。
    rviz_file = os.path.join(package_share, "rviz", "rviz.rviz")  # 拼出 RViz 配置文件的完整路径。

    use_gui = LaunchConfiguration("use_gui")  # 读取是否启动关节状态 GUI 的启动参数。
    use_rviz = LaunchConfiguration('rviz')  # 读取是否启动 RViz 的启动参数。
    use_sim_time = LaunchConfiguration("use_sim_time")  # 读取是否使用仿真时间的启动参数。

    robot_description = ParameterValue(  # 创建 robot_description 参数的值。
        Command(["cat ", urdf_file]),  # 使用 cat 命令读取 URDF 文件内容。
        value_type=str,  # 明确告诉 ROS 2 这个参数是字符串类型。
    )  # 结束 robot_description 参数值的定义。

    declare_use_gui = DeclareLaunchArgument(  # 声明是否启动关节状态 GUI 的参数。
        "use_gui",  # 启动参数的名字。
        default_value="false",  # 默认不启动 GUI，使用普通 joint_state_publisher。
        description="Start the joint state publisher GUI",  # 这个参数在命令行帮助中的说明。
    )  # 结束 use_gui 参数声明。
    declare_rviz = DeclareLaunchArgument(  # 声明是否启动 RViz 的参数。
        'rviz',  # 启动参数的名字。
        default_value='true',  # 默认启动 RViz，保持原有行为。
        description='Start RViz',  # 这个参数在命令行帮助中的说明。
    )  # 结束 rviz 参数声明。
    declare_use_sim_time = DeclareLaunchArgument(  # 声明是否使用仿真时间的参数。
        "use_sim_time",  # 启动参数的名字。
        default_value="false",  # 默认使用系统时间，而不是仿真时间 /clock。
        description="Use simulation time",  # 这个参数在命令行帮助中的说明。
    )  # 结束 use_sim_time 参数声明。

    robot_state_publisher = Node(  # 创建 robot_state_publisher 节点。
        package="robot_state_publisher",  # 指定节点所属的软件包。
        executable="robot_state_publisher",  # 指定要运行的可执行程序。
        name="robot_state_publisher",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        parameters=[  # 开始设置节点参数。
            {  # 使用字典传入多个参数。
                "robot_description": robot_description,  # 传入机器人的 URDF 描述。
                "use_sim_time": use_sim_time,  # 设置是否使用仿真时间。
            }  # 结束节点参数字典。
        ],  # 结束参数列表。
    )  # 结束 robot_state_publisher 节点定义。

    joint_state_publisher = Node(  # 创建普通的 joint_state_publisher 节点。
        package="joint_state_publisher",  # 指定节点所属的软件包。
        executable="joint_state_publisher",  # 指定要运行的可执行程序。
        name="joint_state_publisher",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        condition=UnlessCondition(use_gui),  # 只有 use_gui 为 false 时才启动此节点。
        parameters=[  # 开始设置节点参数。
            {  # 使用字典传入多个参数。
                "robot_description": robot_description,  # 传入机器人的 URDF 描述。
                "use_sim_time": use_sim_time,  # 设置是否使用仿真时间。
            }  # 结束节点参数字典。
        ],  # 结束参数列表。
    )  # 结束普通 joint_state_publisher 节点定义。

    joint_state_publisher_gui = Node(  # 创建带图形界面的关节状态发布节点。
        package="joint_state_publisher_gui",  # 指定节点所属的软件包。
        executable="joint_state_publisher_gui",  # 指定要运行的可执行程序。
        name="joint_state_publisher_gui",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        condition=IfCondition(use_gui),  # 只有 use_gui 为 true 时才启动此节点。
        parameters=[  # 开始设置节点参数。
            {  # 使用字典传入多个参数。
                "robot_description": robot_description,  # 传入机器人的 URDF 描述。
                "use_sim_time": use_sim_time,  # 设置是否使用仿真时间。
            }  # 结束节点参数字典。
        ],  # 结束参数列表。
    )  # 结束 GUI joint_state_publisher 节点定义。

    rviz = Node(  # 创建 RViz2 可视化节点。
        package="rviz2",  # 指定节点所属的软件包。
        executable="rviz2",  # 指定要运行的可执行程序。
        name="rviz2",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        condition=IfCondition(use_rviz),  # 只有 rviz 为 true 时才启动此节点。
        arguments=["-d", rviz_file],  # 加载指定的 RViz 配置文件。
    )  # 结束 RViz2 节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_use_gui)  # 将 use_gui 参数声明加入 launch 描述。
    ld.add_action(declare_rviz)  # 将 rviz 参数声明加入 launch 描述。
    ld.add_action(declare_use_sim_time)  # 将 use_sim_time 参数声明加入 launch 描述。
    ld.add_action(robot_state_publisher)  # 将 robot_state_publisher 节点加入 launch 描述。
    ld.add_action(joint_state_publisher)  # 将普通关节状态发布节点加入 launch 描述。
    ld.add_action(joint_state_publisher_gui)  # 将 GUI 关节状态发布节点加入 launch 描述。
    ld.add_action(rviz)  # 将 RViz2 节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
