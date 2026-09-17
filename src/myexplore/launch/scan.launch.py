import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 从本包 share 目录读参数，保证装到 install/ 之后依然能找到。
    params_file = os.path.join(
        get_package_share_directory("myexplore"), "config", "scan.yaml"
    )

    scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[params_file, {"use_sim_time": False}],
        remappings=[
            # 话题名只由 multi_topic 决定：0 → 固定名 "livox/lidar"；1 → "livox/lidar_<ip>"。
            # 相对名 + 节点在根命名空间，故实际为 /livox/lidar。见 lddc.cpp:644-668。
            # xfer_format 不影响话题名，它只决定消息类型（0=PointCloud2，1=CustomMsg）。
            ("cloud_in", "/livox/lidar"),
            ("scan", "/scan"),
        ],
    )

    return LaunchDescription([scan_node])
