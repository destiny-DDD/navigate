import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 导入参数声明和包含其他 launch 的动作。
from launch.conditions import IfCondition  # 导入根据启动参数决定是否执行动作的条件。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入 Python launch 文件描述对象。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    mynav_share = get_package_share_directory("mynav")  # 获取 mynav 软件包的 share 目录。
    launch_dir = os.path.join(mynav_share, "launch")  # 拼出本包 launch 文件所在的目录。

    def include(name, condition=None):  # 定义复用本包子 launch 文件的辅助函数。
        return IncludeLaunchDescription(  # 创建包含其他 launch 文件的动作。
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),  # 指定子 launch 文件路径。
            condition=condition,  # 传递可选的执行条件。
        )  # 返回子 launch 包含动作。

    # explore:=false 时只启动导航而不启动探索，用于单点导航验证。
    explore_arg = DeclareLaunchArgument(  # 声明是否启动 explore_lite 的参数。
        "explore",  # 启动参数的名字。
        default_value="true",  # 默认启动自动探索。
        description="是否启动 explore_lite；验证 Nav2 单点导航时设为 false",  # 这个参数在命令行帮助中的说明。
    )  # 结束 explore 参数声明。

    scan_launch = include("scan.launch.py")  # 创建启动点云转激光的动作。
    slam_launch = include("slam.launch.py")  # 创建启动 SLAM 的动作。
    navigation_launch = include("navigation.launch.py")  # 创建启动 Nav2 的动作。
    explore_launch = include(  # 创建按参数条件启动 explore_lite 的动作。
        "explore.launch.py",  # 指定自动探索 launch 文件。
        IfCondition(LaunchConfiguration("explore")),  # 只有 explore 为 true 时才执行。
    )  # 结束 explore_lite launch 包含动作。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(explore_arg)  # 将 explore 参数声明加入 launch 描述。
    ld.add_action(scan_launch)  # 将点云转激光动作加入 launch 描述。
    ld.add_action(slam_launch)  # 将 SLAM 动作加入 launch 描述。
    ld.add_action(navigation_launch)  # 将 Nav2 导航动作加入 launch 描述。
    ld.add_action(explore_launch)  # 将条件探索动作加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
