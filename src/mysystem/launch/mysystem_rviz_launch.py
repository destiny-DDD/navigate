import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, GroupAction  # 导入启动参数与动作分组工具。
from launch.conditions import IfCondition, UnlessCondition  # 导入根据参数决定是否启动节点的条件。
from launch.substitutions import Command, LaunchConfiguration  # 导入命令替换和启动参数读取工具。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。
from launch_ros.parameter_descriptions import ParameterValue  # 导入 ROS 2 参数值描述对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    package_share = get_package_share_directory('mysystem')  # 获取 mysystem 软件包的 share 目录。
    urdf_file = os.path.join(package_share, 'urdf', 'robot.urdf')  # 拼出机器人 URDF 文件的完整路径。
    rviz_file = os.path.join(package_share, 'rviz', 'rviz.rviz')  # 拼出 RViz 配置文件的完整路径。

    use_gui = LaunchConfiguration('use_gui')  # 读取是否使用关节状态图形界面的参数。
    use_sim_time = LaunchConfiguration('use_sim_time')  # 读取是否使用仿真时间的参数。
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')  # 读取是否发布手动关节状态的参数。

    robot_description = ParameterValue(  # 创建 robot_description 参数值。
        Command(['cat ', urdf_file]),  # 读取机器人 URDF 文件内容。
        value_type=str,  # 明确 robot_description 是字符串类型。
    )  # 结束 robot_description 参数值定义。

    declare_use_gui = DeclareLaunchArgument(  # 声明是否使用关节状态 GUI 的参数。
        'use_gui',  # 指定启动参数名称。
        default_value='false',  # 默认使用普通手动关节状态发布器。
        description='Use the GUI when manually publishing joint states.',  # 说明 GUI 的用途。
    )  # 结束 use_gui 参数声明。
    declare_use_sim_time = DeclareLaunchArgument(  # 声明是否使用仿真时间的参数。
        'use_sim_time',  # 指定启动参数名称。
        default_value='false',  # 默认使用系统时间。
        description='Use simulation time.',  # 说明仿真时间的用途。
    )  # 结束 use_sim_time 参数声明。
    declare_use_joint_state_publisher = DeclareLaunchArgument(  # 声明是否发布手动关节状态的参数。
        'use_joint_state_publisher',  # 指定启动参数名称。
        default_value='false',  # 默认由 Gazebo 发布真实车轮关节状态。
        description='Publish manual joint states only when Gazebo is not running.',  # 说明手动发布器仅用于非仿真场景。
    )  # 结束 use_joint_state_publisher 参数声明。

    robot_state_publisher = Node(  # 创建 robot_state_publisher 节点。
        package='robot_state_publisher',  # 指定节点所属的软件包。
        executable='robot_state_publisher',  # 指定要运行的可执行程序。
        name='robot_state_publisher',  # 指定节点在 ROS 图中的名称。
        output='screen',  # 将节点日志输出到当前终端。
        parameters=[  # 设置节点参数。
            {  # 传入机器人描述和时间配置。
                'robot_description': robot_description,  # 传入 URDF 机器人描述。
                'use_sim_time': use_sim_time,  # 设置是否使用仿真时间。
            },  # 结束参数字典。
        ],  # 结束参数列表。
    )  # 结束 robot_state_publisher 节点定义。

    joint_state_publisher = Node(  # 创建普通手动关节状态发布节点。
        package='joint_state_publisher',  # 指定节点所属的软件包。
        executable='joint_state_publisher',  # 指定要运行的可执行程序。
        name='joint_state_publisher',  # 指定节点在 ROS 图中的名称。
        output='screen',  # 将节点日志输出到当前终端。
        condition=UnlessCondition(use_gui),  # 未使用 GUI 时启动普通发布器。
        parameters=[  # 设置节点参数。
            {  # 传入机器人描述和时间配置。
                'robot_description': robot_description,  # 传入 URDF 机器人描述。
                'use_sim_time': use_sim_time,  # 设置是否使用仿真时间。
            },  # 结束参数字典。
        ],  # 结束参数列表。
    )  # 结束普通手动关节状态发布节点定义。

    joint_state_publisher_gui = Node(  # 创建手动关节状态图形界面节点。
        package='joint_state_publisher_gui',  # 指定节点所属的软件包。
        executable='joint_state_publisher_gui',  # 指定要运行的可执行程序。
        name='joint_state_publisher_gui',  # 指定节点在 ROS 图中的名称。
        output='screen',  # 将节点日志输出到当前终端。
        condition=IfCondition(use_gui),  # 使用 GUI 时启动图形发布器。
        parameters=[  # 设置节点参数。
            {  # 传入机器人描述和时间配置。
                'robot_description': robot_description,  # 传入 URDF 机器人描述。
                'use_sim_time': use_sim_time,  # 设置是否使用仿真时间。
            },  # 结束参数字典。
        ],  # 结束参数列表。
    )  # 结束图形手动关节状态发布节点定义。

    manual_joint_state_publishers = GroupAction(  # 创建可选的手动关节状态发布动作组。
        actions=[  # 将两种手动发布器加入动作组。
            joint_state_publisher,  # 未使用 GUI 时发布手动关节状态。
            joint_state_publisher_gui,  # 使用 GUI 时发布手动关节状态。
        ],  # 结束动作组内容。
        condition=IfCondition(use_joint_state_publisher),  # 仅明确要求时启用手动发布器。
    )  # 结束手动关节状态发布动作组定义。

    rviz = Node(  # 创建 RViz 节点。
        package='rviz2',  # 指定节点所属的软件包。
        executable='rviz2',  # 指定 RViz 可执行程序。
        name='rviz2',  # 指定节点在 ROS 图中的名称。
        output='screen',  # 将节点日志输出到当前终端。
        arguments=['-d', rviz_file],  # 指定要加载的 RViz 配置文件。
    )  # 结束 RViz 节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_use_gui)  # 将 use_gui 参数声明加入 launch 描述。
    ld.add_action(declare_use_sim_time)  # 将 use_sim_time 参数声明加入 launch 描述。
    ld.add_action(declare_use_joint_state_publisher)  # 将手动关节状态参数声明加入 launch 描述。
    ld.add_action(robot_state_publisher)  # 将动态 TF 发布节点加入 launch 描述。
    ld.add_action(manual_joint_state_publishers)  # 加入可选的手动关节状态发布动作组。
    ld.add_action(rviz)  # 将 RViz 节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
