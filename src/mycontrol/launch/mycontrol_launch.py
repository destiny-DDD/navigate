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
    point_lio_share = get_package_share_directory("point_lio")  # 获取 Point-LIO 软件包的 share 目录。
    libxr_share = get_package_share_directory("ros2_libxr")
    system_share = get_package_share_directory("mysystem")

    livox_msg_launch_file = os.path.join(  # 拼出 MID360 CustomMsg 驱动 launch 文件的完整路径。
        livox_share,  # Livox 软件包的 share 目录。
        "launch_ROS2",  # ROS 2 launch 文件所在的子目录。
        "msg_MID360_launch.py",  # 发布 /livox/lidar 和 /livox/imu 的 MID360 启动文件。
    )  # 结束 MID360 launch 文件路径拼接。
    point_lio_launch_file = os.path.join(  # 拼出 Point-LIO launch 文件的完整路径。
        point_lio_share,  # Point-LIO 软件包的 share 目录。
        "launch",  # launch 文件所在的子目录。
        "point_lio.launch.py",  # Point-LIO 启动文件。
    )  # 结束 Point-LIO launch 文件路径拼接。
    libxr_launch_file = os.path.join(
        libxr_share,
        "launch",
        "ros2_libxr_launch.py",
    )
    system_launch_file = os.path.join(
        system_share,
        "launch",
        "mysystem_rviz_launch.py",
    )

    livox_msg_launch = IncludeLaunchDescription(  # 创建包含 MID360 驱动的启动动作。
        PythonLaunchDescriptionSource(livox_msg_launch_file),  # 指定 MID360 CustomMsg launch 文件。
    )  # 结束 MID360 驱动启动动作。
    point_lio_launch = IncludeLaunchDescription(  # 创建包含 Point-LIO 的启动动作。
        PythonLaunchDescriptionSource(point_lio_launch_file),  # 指定 Point-LIO launch 文件。
        launch_arguments={"rviz":"false"}.items(),
    )  # 结束 Point-LIO launch 包含动作。
    libxr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(libxr_launch_file),
        launch_arguments={"vid":"16d0","pid":"1492"}.items(),
    )
    system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(system_launch_file),
        launch_arguments={'rviz': 'true'}.items(),
    )
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("cloud_in", "/cloud_registered_body"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "base_link",
            "min_height": 0.05,
            "max_height": 0.35,
            "range_min": 0.15,
            "range_max": 2.5,
            "use_inf": True,
        }],
    )

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(livox_msg_launch)  # 先启动 MID360 驱动，为 Point-LIO 发布原始点云和 IMU。
    ld.add_action(point_lio_launch)  # 将 Point-LIO 启动动作加入 launch 描述。
    ld.add_action(libxr_launch)
    ld.add_action(system_launch)
    ld.add_action(pointcloud_to_laserscan)

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
