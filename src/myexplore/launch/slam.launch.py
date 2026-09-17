import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    myexplore_share = get_package_share_directory("myexplore")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_slam_params_file = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(myexplore_share, "config", "slam_toolbox.yaml"),
        description="slam_toolbox 参数文件路径",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="真机固定 false；上游 launch 默认 true，必须显式覆盖",
    )

    # 复用上游 launch 而非自己起 LifecycleNode：它已处理好 configure/activate 的
    # 状态迁移事件链（online_async_launch.py:55-77），手写容易漏掉导致节点停在 inactive。
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    return LaunchDescription([
        declare_slam_params_file,
        declare_use_sim_time,
        slam,
    ])
