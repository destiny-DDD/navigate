"""启动静态地图定位与 Nav2 导航服务器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mynav_share = get_package_share_directory("mynav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params_file = os.path.join(mynav_share, "params", "nav2_params.yaml")
    default_map_file = os.path.join(mynav_share, "maps", "first_map.yaml")
    default_pcd_file = os.path.join(mynav_share, "PCD", "scans.pcd")
    bringup_launch_file = os.path.join(
        nav2_bringup_share, "launch", "bringup_launch.py"
    )

    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_localization = LaunchConfiguration("use_localization")
    pcd_file = LaunchConfiguration("pcd_file")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=default_map_file,
        description="静态地图 YAML 文件的完整路径",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="完整的 Nav2 参数文件路径",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="是否使用仿真时间",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="是否自动激活 Nav2 生命周期节点",
    )
    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="True",
        description="是否启动 map_server 和 AMCL 重定位",
    )
    declare_pcd_file = DeclareLaunchArgument(
        "pcd_file",
        default_value=default_pcd_file,
        description="要发布的 PCD 点云文件路径",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "slam": "False",
            "use_localization": use_localization,
        }.items(),
    )

    pointcloud_publisher = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="pcd_publisher",
        output="screen",
        parameters=[
            {
                "file_name": pcd_file,
                "tf_frame": "map",
                "publishing_period_ms": 1000,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_localization)
    ld.add_action(declare_pcd_file)
    ld.add_action(nav2_bringup)
    ld.add_action(pointcloud_publisher)

    return ld
