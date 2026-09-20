import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    mynav_share = get_package_share_directory("mynav")  # 获取 mynav 软件包的 share 目录。
    params_file = os.path.join(  # 拼出 explore_lite 参数文件的完整路径。
        mynav_share,  # mynav 软件包的 share 目录。
        "config",  # 配置文件所在的子目录。
        "explore_params.yaml",  # explore_lite 参数文件名。
    )  # 结束参数文件路径拼接。

    # 不用上游自带的 explore.launch.py：它把参数文件写死为 params.yaml，
    # 数据源是 /map，增量更新路径不通，且 use_sim_time 默认 true。
    explore_node = Node(  # 创建 explore_lite 自动探索节点。
        package="explore_lite",  # 指定节点所属的软件包。
        executable="explore",  # 指定要运行的可执行程序。
        name="explore_node",  # 节点名必须匹配 explore_params.yaml 的根键。
        output="screen",  # 将节点日志输出到当前终端。
        parameters=[params_file],  # 加载本包提供的探索参数文件。
    )  # 结束 explore_lite 节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(explore_node)  # 将 explore_lite 节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
