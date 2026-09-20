import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("mynav"), "config", "explore_params.yaml"
    )

    # 不用上游自带的 explore.launch.py：它把参数文件写死为 params.yaml
    # （数据源是 /map，增量更新路径不通），且 use_sim_time 默认 true。
    explore_node = Node(
        package="explore_lite",
        executable="explore",
        # name 必须是 explore_node，才能匹配 explore_params.yaml 的根键。
        name="explore_node",
        output="screen",
        parameters=[params_file],
    )

    return LaunchDescription([explore_node])
