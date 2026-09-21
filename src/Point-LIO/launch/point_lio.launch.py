import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument  # 导入“声明启动参数”的动作。
from launch.conditions import IfCondition  # 导入根据参数决定是否启动节点的条件。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    package_share = get_package_share_directory("point_lio")  # 获取 point_lio 软件包的 share 目录。
    default_config_file = os.path.join(package_share, "config", "mid360.yaml")  # 拼出默认 Point-LIO 配置文件的完整路径。
    rviz_config_file = os.path.join(package_share, "rviz_cfg", "rviz.rviz")  # 拼出 RViz 配置文件的完整路径。

    namespace = LaunchConfiguration("namespace")  # 读取节点命名空间启动参数。
    use_rviz = LaunchConfiguration("rviz")  # 读取是否启动 RViz 的启动参数。
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")  # 读取 Point-LIO 配置文件路径启动参数。

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]  # 将全局 TF 话题重映射到当前命名空间。

    declare_namespace = DeclareLaunchArgument(  # 声明节点命名空间参数。
        "namespace",  # 启动参数的名字。
        default_value="",  # 默认不使用命名空间。
        description="Namespace for the node",  # 这个参数在命令行帮助中的说明。
    )  # 结束 namespace 参数声明。
    declare_rviz = DeclareLaunchArgument(  # 声明是否启动 RViz 的参数。
        "rviz",  # 启动参数的名字。
        default_value="True",  # 默认启动 RViz。
        description="Flag to launch RViz.",  # 这个参数在命令行帮助中的说明。
    )  # 结束 rviz 参数声明。
    declare_point_lio_cfg_dir = DeclareLaunchArgument(  # 声明 Point-LIO 配置文件路径参数。
        "point_lio_cfg_dir",  # 启动参数的名字。
        default_value=default_config_file,  # 默认使用 MID360 配置文件。
        description="Path to the Point-LIO config file",  # 这个参数在命令行帮助中的说明。
    )  # 结束 point_lio_cfg_dir 参数声明。

    start_point_lio_node = Node(  # 创建 Point-LIO 建图节点。
        package="point_lio",  # 指定节点所属的软件包。
        executable="pointlio_mapping",  # 指定要运行的可执行程序。
        namespace=namespace,  # 设置节点命名空间。
        parameters=[point_lio_cfg_dir],  # 传入 Point-LIO 配置文件。
        remappings=remappings,  # 应用 TF 话题重映射。
        output="screen",  # 将节点日志输出到当前终端。
    )  # 结束 Point-LIO 节点定义。
    start_rviz_node = Node(  # 创建 RViz 可视化节点。
        condition=IfCondition(use_rviz),  # 只有 rviz 为 true 时才启动此节点。
        package="rviz2",  # 指定节点所属的软件包。
        executable="rviz2",  # 指定要运行的可执行程序。
        namespace=namespace,  # 设置节点命名空间。
        name="rviz",  # 指定节点在 ROS 图中的名字。
        remappings=remappings,  # 应用 TF 话题重映射。
        arguments=[  # 设置 RViz 的命令行参数。
            "-d",  # 指定要加载的 RViz 配置文件。
            rviz_config_file,  # 传入 RViz 配置文件的完整路径。
        ],  # 结束 RViz 命令行参数。
    )  # 结束 RViz 节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_namespace)  # 将 namespace 参数声明加入 launch 描述。
    ld.add_action(declare_rviz)  # 将 rviz 参数声明加入 launch 描述。
    ld.add_action(declare_point_lio_cfg_dir)  # 将配置文件路径参数声明加入 launch 描述。
    ld.add_action(start_point_lio_node)  # 将 Point-LIO 节点加入 launch 描述。
    ld.add_action(start_rviz_node)  # 将 RViz 节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
