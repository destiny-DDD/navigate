import os  # 导入操作系统相关工具，用来拼接文件路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch_ros.actions import Node  # 导入用于启动 ROS 2 节点的对象。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    # 从本包 share 目录读取参数，保证安装到 install/ 后依然能找到。
    mynav_share = get_package_share_directory("mynav")  # 获取 mynav 软件包的 share 目录。
    params_file = os.path.join(  # 拼出点云转激光参数文件的完整路径。
        mynav_share,  # mynav 软件包的 share 目录。
        "config",  # 配置文件所在的子目录。
        "scan.yaml",  # 点云转激光参数文件名。
    )  # 结束参数文件路径拼接。

    scan_node = Node(  # 创建点云转激光扫描节点。
        package="pointcloud_to_laserscan",  # 指定节点所属的软件包。
        executable="pointcloud_to_laserscan_node",  # 指定要运行的可执行程序。
        name="pointcloud_to_laserscan",  # 指定节点在 ROS 图中的名字。
        output="screen",  # 将节点日志输出到当前终端。
        parameters=[params_file, {"use_sim_time": False}],  # 加载参数文件并固定使用系统时间。
        remappings=[  # 设置输入输出话题重映射。
            # 话题名只由 multi_topic 决定：0 为固定名 livox/lidar，1 为 livox/lidar_<ip>。
            # 相对名加根命名空间后实际为 /livox/lidar；xfer_format 只决定消息类型。
            ("cloud_in", "/livox/lidar"),  # 将 Livox 点云话题接入节点输入。
            ("scan", "/scan"),  # 将节点输出接到导航使用的激光话题。
        ],  # 结束话题重映射列表。
    )  # 结束点云转激光节点定义。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(scan_node)  # 将点云转激光节点加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
