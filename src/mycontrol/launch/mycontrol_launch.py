import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 导入参数声明和子 launch 包含动作。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入 Python launch 文件描述对象。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。
from launch_ros.parameter_descriptions import ParameterValue  # 导入 ROS 2 参数值描述对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    vid = LaunchConfiguration("vid")  # 读取串口设备的 USB 厂商 ID 启动参数。
    pid = LaunchConfiguration("pid")  # 读取串口设备的 USB 产品 ID 启动参数。

    declare_vid = DeclareLaunchArgument(  # 声明串口厂商 ID 的启动参数。
        "vid",  # 启动参数的名字。
        default_value="16d0",  # 默认值，对应 CH340 串口芯片。
        description="USB vendor ID of the chassis serial port",  # 这个参数在命令行帮助中的说明。
    )  # 结束 vid 参数声明。
    declare_pid = DeclareLaunchArgument(  # 声明串口产品 ID 的启动参数。
        "pid",  # 启动参数的名字。
        default_value="1492",  # 默认值，对应 CH340 串口芯片。
        description="USB product ID of the chassis serial port",  # 这个参数在命令行帮助中的说明。
    )  # 结束 pid 参数声明。
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

    control_node = Node(  # 创建底盘控制节点，也就是 main.cpp 编译出来的可执行程序。
        package="mycontrol",  # 指定节点所属的软件包。
        executable="mycontrol",  # 指定要运行的可执行程序，名字来自 CMakeLists 里的 add_executable。
        name="control_node",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        parameters=[  # 开始设置节点参数。
            {  # 使用字典传入多个参数。
                # 必须用 ParameterValue 显式声明成字符串。直接写 vid / pid 的话，
                # launch_ros 会把 "7523" 这种长得像数字的值转成整数，而节点里
                # declare_parameter<std::string> 声明的是字符串，类型对不上会直接
                # 抛 InvalidParameterTypeException 让节点 abort。
                "vid": ParameterValue(vid, value_type=str),  # 串口厂商 ID，保持字符串类型。
                "pid": ParameterValue(pid, value_type=str),  # 串口产品 ID，保持字符串类型。
            }  # 结束节点参数字典。
        ],  # 结束参数列表。
    )  # 结束底盘控制节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_vid)  # 将 vid 参数声明加入 launch 描述。
    ld.add_action(declare_pid)  # 将 pid 参数声明加入 launch 描述。
    ld.add_action(livox_msg_launch)  # 先启动 MID360 驱动，为 Point-LIO 发布原始点云和 IMU。
    ld.add_action(point_lio_launch)  # 将 Point-LIO 启动动作加入 launch 描述。
    ld.add_action(control_node)  # 将底盘控制节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
