import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 导入参数声明和包含其他 launch 的动作。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入 Python launch 文件描述对象。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    mynav_share = get_package_share_directory("mynav")  # 获取 mynav 软件包的 share 目录。
    nav2_bringup_share = get_package_share_directory("nav2_bringup")  # 获取 Nav2 bringup 软件包的 share 目录。

    params_file = LaunchConfiguration("params_file")  # 读取完整的 Nav2 参数文件路径。
    use_sim_time = LaunchConfiguration("use_sim_time")  # 读取是否使用仿真时间的启动参数。
    autostart = LaunchConfiguration("autostart")  # 读取是否自动激活生命周期节点的启动参数。

    declare_params_file = DeclareLaunchArgument(  # 声明 Nav2 参数文件路径。
        "params_file",  # 启动参数的名字。
        default_value=os.path.join(mynav_share, "config", "nav2_params.yaml"),  # 默认使用 mynav 包中的参数文件。
        description="Nav2 参数文件路径",  # 这个参数在命令行帮助中的说明。
    )  # 结束 params_file 参数声明。
    declare_use_sim_time = DeclareLaunchArgument(  # 声明是否使用仿真时间的参数。
        "use_sim_time",  # 启动参数的名字。
        default_value="false",  # 真机默认使用系统时间。
        description="真机固定 false",  # 这个参数在命令行帮助中的说明。
    )  # 结束 use_sim_time 参数声明。
    declare_autostart = DeclareLaunchArgument(  # 声明是否自动激活各生命周期节点的参数。
        "autostart",  # 启动参数的名字。
        default_value="true",  # 默认自动激活导航节点。
        description="自动激活各生命周期节点",  # 这个参数在命令行帮助中的说明。
    )  # 结束 autostart 参数声明。

    # 关键：用 navigation_launch.py 而**不是** bringup_launch.py。
    #
    # bringup_launch.py 并不是"没有 slam"——它两条路都有，靠一组真值表二选一：
    #   slam and use_localization       → include slam_launch.py       (bringup_launch.py:156-160)
    #   not slam and use_localization   → include localization_launch.py (:169-173)
    # 其中 slam 默认 False（:95-96），所以不显式传参时走的是 AMCL 那条路。
    #
    # 不用它的实际原因有两条，都与"有没有 slam"无关：
    #   1) slam_launch.py:114-140 会把你的 params_file 整份转交给 slam_toolbox，
    #      要求把 slam 参数并进 nav2_params.yaml；我们两者是独立文件。
    #   2) slam_launch.py:44 用的是 online_sync（同步），我们主动选了 online_async。
    #
    # 而 navigation_launch.py 只含导航栈本体，不含 amcl / map_server —— 正好让我们
    # 用自己那份 slam.launch.py 顶上那个二选一的位置，slam 与 nav2 彻底解耦。
    # 附带好处：collision_monitor 与 velocity_smoother 已由它拉起，无需另行启动。
    navigation_launch_file = os.path.join(  # 拼出 Nav2 官方导航 launch 文件的完整路径。
        nav2_bringup_share,  # Nav2 bringup 软件包的 share 目录。
        "launch",  # launch 文件所在的子目录。
        "navigation_launch.py",  # Nav2 导航 launch 文件名。
    )  # 结束 Nav2 launch 文件路径拼接。
    nav2_navigation = IncludeLaunchDescription(  # 创建包含 Nav2 官方导航 launch 的动作。
        PythonLaunchDescriptionSource(navigation_launch_file),  # 指定 Nav2 官方导航 launch 文件。
        launch_arguments={  # 向 Nav2 官方 launch 传递启动参数。
            "params_file": params_file,  # 传入完整的 Nav2 参数文件。
            "use_sim_time": use_sim_time,  # 传递是否使用仿真时间的设置。
            "autostart": autostart,  # 传递是否自动激活生命周期节点的设置。
        }.items(),  # 将参数字典转换为 launch 参数迭代器。
    )  # 结束 Nav2 官方导航 launch 包含动作。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_params_file)  # 将 Nav2 参数文件声明加入 launch 描述。
    ld.add_action(declare_use_sim_time)  # 将仿真时间参数声明加入 launch 描述。
    ld.add_action(declare_autostart)  # 将自动激活参数声明加入 launch 描述。
    ld.add_action(nav2_navigation)  # 将 Nav2 导航动作加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
