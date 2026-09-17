import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument  # 导入“声明启动参数”的动作。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。
from launch_ros.parameter_descriptions import ParameterValue  # 导入 ROS 2 参数值描述对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    # 注意：odin 的 host_sdk_sample 节点不读取任何 ROS 参数，它的配置全部来自
    # config/control_command.yaml，路径由节点自己推导（见 host_sdk_sample.cpp 中
    # get_package_source_directory() 那一段）。所以这里不需要、也没法通过参数传
    # 配置文件路径，要调 odin 的设置请直接改那个 yaml 文件。

    livox_share = get_package_share_directory("livox_ros_driver2")  # 获取 Livox 驱动软件包的 share 目录。
    livox_user_config = os.path.join(  # 拼出 MID360 的网络配置文件完整路径。
        livox_share,  # Livox 软件包的 share 目录。
        "config",  # 配置文件所在的子目录。
        "MID360_config.json",  # 里面写着雷达和主机的 IP、端口，连不上雷达时先查这个文件。
    )  # 结束 MID360 配置文件路径拼接。

    vid = LaunchConfiguration("vid")  # 读取串口设备的 USB 厂商 ID 启动参数。
    pid = LaunchConfiguration("pid")  # 读取串口设备的 USB 产品 ID 启动参数。
    frame_id = LaunchConfiguration("frame_id")  # 读取点云消息的参考坐标系启动参数。
    publish_freq = LaunchConfiguration("publish_freq")  # 读取点云发布频率启动参数。

    declare_vid = DeclareLaunchArgument(  # 声明串口厂商 ID 的启动参数。
        "vid",  # 启动参数的名字。
        default_value="1a86",  # 默认值，对应 CH340 串口芯片。
        description="USB vendor ID of the chassis serial port",  # 这个参数在命令行帮助中的说明。
    )  # 结束 vid 参数声明。
    declare_pid = DeclareLaunchArgument(  # 声明串口产品 ID 的启动参数。
        "pid",  # 启动参数的名字。
        default_value="7523",  # 默认值，对应 CH340 串口芯片。
        description="USB product ID of the chassis serial port",  # 这个参数在命令行帮助中的说明。
    )  # 结束 pid 参数声明。
    declare_frame_id = DeclareLaunchArgument(  # 声明点云坐标系名称的启动参数。
        "frame_id",  # 启动参数的名字。
        default_value="livox_frame",  # 默认值与 mysystem 的 URDF 中雷达 link 的名字保持一致。
        description="Frame id of the MID360 point cloud",  # 这个参数在命令行帮助中的说明。
    )  # 结束 frame_id 参数声明。
    declare_publish_freq = DeclareLaunchArgument(  # 声明点云发布频率的启动参数。
        "publish_freq",  # 启动参数的名字。
        default_value="10.0",  # 默认 10 Hz，可改成 5.0、20.0、50.0 等。
        description="Point cloud publish frequency in Hz",  # 这个参数在命令行帮助中的说明。
    )  # 结束 publish_freq 参数声明。

    odin_node = Node(  # 创建 odin 雷达主驱动节点。
        package="odin_ros_driver",  # 指定节点所属的软件包。
        executable="host_sdk_sample",  # 指定要运行的可执行程序。
        name="host_sdk_sample",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        # 这里故意不传 parameters：该节点不读任何 ROS 参数，
        # 上游自己的 launch 传了 config_file 但代码里并没有实现读取，
        # 传了也是静默忽略，所以干脆不写，避免误导。
    )  # 结束 odin 节点定义。

    livox_node = Node(  # 创建 MID360 激光雷达驱动节点。
        package="livox_ros_driver2",  # 指定节点所属的软件包。
        executable="livox_ros_driver2_node",  # 指定要运行的可执行程序。
        name="livox_lidar_publisher",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        parameters=[  # 开始设置节点参数。
            {"xfer_format": 1},  # 1 表示使用 Livox 自定义点云格式，0 表示标准 PointCloud2。
            {"multi_topic": 0},  # 0 表示所有雷达共用同一个话题。
            {"data_src": 0},  # 0 表示数据来自真实雷达，其他值无意义。
            {"publish_freq": ParameterValue(publish_freq, value_type=float)},  # 点云发布频率，必须转成浮点类型。
            {"output_data_type": 0},  # 点云中是否携带强度等额外字段，0 表示默认。
            {"frame_id": frame_id},  # 点云消息的参考坐标系，要和 URDF 里的 link 名字对应。
            {"lvx_file_path": "/home/livox/livox_test.lvx"},  # 回放离线文件时用的路径，实时模式下不会读到。
            {"user_config_path": livox_user_config},  # MID360 的网络配置文件路径。
            {"cmdline_input_bd_code": "livox0000000001"},  # 雷达的广播码，单雷达时保持默认即可。
        ],  # 结束参数列表。
    )  # 结束 MID360 节点定义。

    control_node = Node(  # 创建底盘控制节点，也就是 main.cpp 编译出来的可执行程序。
        package="mycontrol",  # 指定节点所属的软件包。
        executable="mycontrol",  # 指定要运行的可执行程序，名字来自 CMakeLists 里的 add_executable。
        name="control_node",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        parameters=[  # 开始设置节点参数。
            {  # 使用字典传入多个参数。
                "vid": vid,  # 传给节点内部 declare_parameter("vid", ...) 的串口厂商 ID。
                "pid": pid,  # 传给节点内部 declare_parameter("pid", ...) 的串口产品 ID。
            }  # 结束节点参数字典。
        ],  # 结束参数列表。
    )  # 结束底盘控制节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_vid)  # 将 vid 参数声明加入 launch 描述。
    ld.add_action(declare_pid)  # 将 pid 参数声明加入 launch 描述。
    ld.add_action(declare_frame_id)  # 将 frame_id 参数声明加入 launch 描述。
    ld.add_action(declare_publish_freq)  # 将 publish_freq 参数声明加入 launch 描述。
    ld.add_action(odin_node)  # 将 odin 雷达驱动节点加入 launch 描述。
    ld.add_action(livox_node)  # 将 MID360 雷达驱动节点加入 launch 描述。
    ld.add_action(control_node)  # 将底盘控制节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
