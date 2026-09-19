import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import IncludeLaunchDescription  # 导入用于包含其他 launch 文件的动作。
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,  # 导入 Python launch 文件的来源描述对象。
)
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    package_share = get_package_share_directory('mysystem')  # 获取 mysystem 软件包的 share 目录。
    world_file = os.path.join(package_share, 'worlds', 'world.world')  # 拼出 world 文件的完整路径。
    robot_file = os.path.join(package_share, 'urdf', 'robot.urdf')  # 拼出机器人 URDF 文件的完整路径。
    bridge_config = os.path.join(package_share, 'config', 'bridge.yaml')  # 拼出 Gazebo 桥接配置文件的完整路径。
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')  # 获取 ros_gz_sim 软件包的 share 目录。
    gazebo = IncludeLaunchDescription(  # 创建加载 Gazebo Sim 的启动动作。
        PythonLaunchDescriptionSource(  # 指定要包含的 Python launch 文件。
            os.path.join(
                ros_gz_sim_share, 'launch', 'gz_sim.launch.py'
            )  # 拼出 gz_sim.launch.py 的完整路径。
        ),  # 结束 launch 文件来源描述。
        launch_arguments={  # 将启动参数传递给 ros_gz_sim。
            'gz_args': f'-r {world_file}',  # 指定 Gazebo 要加载的 world 文件。
            'on_exit_shutdown': 'true',  # Gazebo 退出时关闭整个 launch 系统。
        }.items(),  # 将字典转换为 launch 参数项。
    )  # 结束 Gazebo Sim 启动动作定义。

    spawn_robot = Node(  # 创建将 URDF 机器人生成到 Gazebo Sim 的节点。
        package='ros_gz_sim',  # 指定节点所属的软件包。
        executable='create',  # 指定用于创建 Gazebo 实体的可执行程序。
        arguments=[  # 设置机器人生成参数。
            '-file', robot_file,  # 指定要加载的机器人 URDF 文件。
            '-name', 'robot',  # 指定 Gazebo 中的实体名称。
            '-x', '0',  # 设置机器人初始 X 坐标。
            '-y', '0',  # 设置机器人初始 Y 坐标。
            '-z', '0.2',  # 让机器人初始位置略高于地面。
        ],  # 结束机器人生成参数列表。
        output='screen',  # 将节点日志输出到当前终端。
    )  # 结束机器人生成节点定义。

    bridge = Node(  # 创建 Gazebo 与 ROS 2 消息桥接节点。
        package='ros_gz_bridge',  # 指定桥接节点所属的软件包。
        executable='parameter_bridge',  # 指定通用消息桥接可执行程序。
        parameters=[  # 设置桥接节点参数。
            {'config_file': bridge_config},  # 从 YAML 文件读取需要桥接的话题。
        ],  # 结束桥接节点参数列表。
        output='screen',  # 将节点日志输出到当前终端。
    )  # 结束消息桥接节点定义。

    odom_to_tf = Node(  # 创建将里程计转换为 TF 的节点。
        package='mysystem',  # 指定节点所属的软件包。
        executable='odom_to_tf',  # 指定安装后的 C++ TF 广播节点。
        output='screen',  # 将节点日志输出到当前终端。
    )  # 结束里程计 TF 节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(gazebo)  # 将 Gazebo Sim 启动动作加入 launch 描述。
    ld.add_action(spawn_robot)  # 将机器人生成节点加入 launch 描述。
    ld.add_action(bridge)  # 将 Gazebo 与 ROS 2 的消息桥接节点加入 launch 描述。
    ld.add_action(odom_to_tf)  # 将里程计 TF 节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
