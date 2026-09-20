import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 导入参数声明和包含其他 launch 的动作。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入 Python launch 文件描述对象。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    mynav_share = get_package_share_directory("mynav")  # 获取 mynav 软件包的 share 目录。
    slam_toolbox_share = get_package_share_directory("slam_toolbox")  # 获取 slam_toolbox 软件包的 share 目录。

    slam_params_file = LaunchConfiguration("slam_params_file")  # 读取 slam_toolbox 参数文件路径。
    use_sim_time = LaunchConfiguration("use_sim_time")  # 读取是否使用仿真时间的启动参数。

    declare_slam_params_file = DeclareLaunchArgument(  # 声明 slam_toolbox 参数文件路径。
        "slam_params_file",  # 启动参数的名字。
        default_value=os.path.join(mynav_share, "config", "slam_toolbox.yaml"),  # 默认使用 mynav 包中的参数文件。
        description="slam_toolbox 参数文件路径",  # 这个参数在命令行帮助中的说明。
    )  # 结束 slam_params_file 参数声明。
    declare_use_sim_time = DeclareLaunchArgument(  # 声明是否使用仿真时间的参数。
        "use_sim_time",  # 启动参数的名字。
        default_value="false",  # 真机默认使用系统时间。
        description="真机固定 false；上游 launch 默认 true，必须显式覆盖",  # 这个参数在命令行帮助中的说明。
    )  # 结束 use_sim_time 参数声明。

    # 复用上游 launch 而非自己起 LifecycleNode：它已处理好 configure/activate 的
    # 状态迁移事件链（online_async_launch.py:55-77），手写容易漏掉导致节点停在 inactive。
    slam_launch_file = os.path.join(  # 拼出 slam_toolbox 异步 launch 文件的完整路径。
        slam_toolbox_share,  # slam_toolbox 软件包的 share 目录。
        "launch",  # launch 文件所在的子目录。
        "online_async_launch.py",  # 异步 SLAM launch 文件名。
    )  # 结束 SLAM launch 文件路径拼接。
    # 复用上游 launch 而不是自己启动 LifecycleNode：它已处理好 configure/activate 的
    # 状态迁移事件链，手写容易遗漏，导致节点停在 inactive 状态。
    slam = IncludeLaunchDescription(  # 创建包含 slam_toolbox 上游 launch 的动作。
        PythonLaunchDescriptionSource(slam_launch_file),  # 指定 slam_toolbox 异步 launch 文件。
        launch_arguments={  # 向 slam_toolbox launch 传递启动参数。
            "slam_params_file": slam_params_file,  # 传入 SLAM 参数文件。
            "use_sim_time": use_sim_time,  # 传递是否使用仿真时间的设置。
            "autostart": "true",  # 自动激活 SLAM 生命周期节点。
            "use_lifecycle_manager": "false",  # 使用上游 launch 自己的状态迁移逻辑。
        }.items(),  # 将参数字典转换为 launch 参数迭代器。
    )  # 结束 slam_toolbox launch 包含动作。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_slam_params_file)  # 将 SLAM 参数文件声明加入 launch 描述。
    ld.add_action(declare_use_sim_time)  # 将仿真时间参数声明加入 launch 描述。
    ld.add_action(slam)  # 将 slam_toolbox 启动动作加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
