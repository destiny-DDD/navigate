import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import IncludeLaunchDescription  # 导入用于包含其他 launch 文件的动作。
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,  # 导入 Python launch 文件的来源描述对象。
)


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    package_share = get_package_share_directory('mysystem')  # 获取 mysystem 软件包的 share 目录。
    world_file = os.path.join(package_share, 'worlds', 'world.world')  # 拼出 world 文件的完整路径。
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')  # 获取 ros_gz_sim 软件包的 share 目录。

    gazebo = IncludeLaunchDescription(  # 创建加载 Gazebo Sim 的启动动作。
        PythonLaunchDescriptionSource(  # 指定要包含的 Python launch 文件。
            os.path.join(
                ros_gz_sim_share, 'launch', 'gz_sim.launch.py'
            )  # 拼出 gz_sim.launch.py 的完整路径。
        ),  # 结束 launch 文件来源描述。
        launch_arguments={  # 将启动参数传递给 ros_gz_sim。
            'gz_args': world_file,  # 指定 Gazebo 要加载的 world 文件。
            'on_exit_shutdown': 'true',  # Gazebo 退出时关闭整个 launch 系统。
        }.items(),  # 将字典转换为 launch 参数项。
    )  # 结束 Gazebo Sim 启动动作定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(gazebo)  # 将 Gazebo Sim 启动动作加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
