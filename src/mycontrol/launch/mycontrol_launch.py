import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 导入参数声明和子 launch 包含动作。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入 Python launch 文件描述对象。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。
from launch_ros.parameter_descriptions import ParameterValue  # 导入 ROS 2 参数值描述对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    livox_share = get_package_share_directory("livox_ros_driver2")  # 获取 Livox 驱动软件包的 share 目录。
    livox_msg_launch_file = os.path.join(  # 拼出 MID360 CustomMsg 驱动 launch 文件的完整路径。
        livox_share,  # Livox 软件包的 share 目录。
        "launch_ROS2",  # ROS 2 launch 文件所在的子目录。
        "msg_MID360_launch.py",  # 发布 /livox/lidar 和 /livox/imu 的 MID360 启动文件。
    )  # 结束 MID360 launch 文件路径拼接。
    livox_msg_launch = IncludeLaunchDescription(  # 创建包含 MID360 驱动的启动动作。
        PythonLaunchDescriptionSource(livox_msg_launch_file),  # 指定 MID360 CustomMsg launch 文件。
    )  # 结束 MID360 驱动启动动作。
    point_lio_share = get_package_share_directory("point_lio")  # 获取 Point-LIO 软件包的 share 目录。
    point_lio_launch_file = os.path.join(  # 拼出 Point-LIO launch 文件的完整路径。
        point_lio_share,  # Point-LIO 软件包的 share 目录。
        "launch",  # launch 文件所在的子目录。
        "point_lio.launch.py",  # Point-LIO 启动文件。
    )  # 结束 Point-LIO launch 文件路径拼接。
    point_lio_launch = IncludeLaunchDescription(  # 创建包含 Point-LIO 的启动动作。
        PythonLaunchDescriptionSource(point_lio_launch_file),  # 指定 Point-LIO launch 文件。
    )  # 结束 Point-LIO launch 包含动作。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(livox_msg_launch)  # 先启动 MID360 驱动，为 Point-LIO 发布原始点云和 IMU。
    ld.add_action(point_lio_launch)  # 将 Point-LIO 启动动作加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
