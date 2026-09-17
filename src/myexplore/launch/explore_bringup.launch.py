import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("myexplore"), "launch")

    def include(name, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            condition=condition,
        )

    # explore:=false 时只起导航不起探索，用于 Task 4 的单点导航验证。
    explore_arg = DeclareLaunchArgument(
        "explore",
        default_value="true",
        description="是否启动 explore_lite；验证 nav2 单点导航时设为 false",
    )

    return LaunchDescription([
        explore_arg,
        include("scan.launch.py"),
        include("slam.launch.py"),
        include("navigation.launch.py"),
        include("explore.launch.py", IfCondition(LaunchConfiguration("explore"))),
    ])
