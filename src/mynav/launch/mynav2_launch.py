"""启动静态地图定位与 Nav2 导航服务器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mynav_share = get_package_share_directory("mynav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    default_params_file = os.path.join(mynav_share, "params", "nav2_params.yaml")
    bringup_launch_file = os.path.join(
        nav2_bringup_share, "launch", "bringup_launch.py"
    )

    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_localization = LaunchConfiguration("use_localization")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="静态地图 YAML 文件的完整路径",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="完整的 Nav2 参数文件路径",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="是否使用仿真时间",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="是否自动激活 Nav2 生命周期节点",
    )
    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="true",
        description="是否启动 map_server 和 AMCL 重定位",
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "slam": "false",
            "use_localization": use_localization,
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_map)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_localization)
    ld.add_action(nav2_bringup)

    return ld
