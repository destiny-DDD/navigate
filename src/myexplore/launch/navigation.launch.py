import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    myexplore_share = get_package_share_directory("myexplore")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(myexplore_share, "config", "nav2_params.yaml"),
        description="nav2 参数文件路径",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="真机固定 false"
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart", default_value="true", description="自动激活各生命周期节点"
    )

    # 关键：用 navigation_launch.py 而**不是** bringup_launch.py。
    #
    # bringup_launch.py 并不是"没有 slam"——它两条路都有，靠一组真值表二选一：
    #   slam and use_localization       → include slam_launch.py       (bringup_launch.py:156-160)
    #   not slam and use_localization   → include localization_launch.py (:169-173)
    # 其中 slam 默认 False（:95-96），所以不显式传参时走的是 AMCL 那条路。
    #
    # 不用它的实际原因有两条，都与"有没有 slam"无关：
    #   1) slam_launch.py:114-140 会把你的 params_file 整份转交给 slam_toolbox，
    #      要求把 slam 参数并进 nav2_params.yaml；我们两者是独立文件。
    #   2) slam_launch.py:44 用的是 online_sync（同步），我们主动选了 online_async。
    #
    # 而 navigation_launch.py 只含导航栈本体，不含 amcl / map_server —— 正好让我们
    # 用自己那份 slam.launch.py 顶上那个二选一的位置，slam 与 nav2 彻底解耦。
    # 附带好处：collision_monitor 与 velocity_smoother 已由它拉起，无需另行启动。
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription([
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        nav2_navigation,
    ])
