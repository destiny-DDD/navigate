"""复用 mynav 导航配置，将 myBT 行为树设为单目标导航默认树."""

import os  # 导入路径工具，用来拼接配置文件和行为树文件的路径。

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    mybt_share = get_package_share_directory('my_bt')  # 获取行为树包的 share 目录。
    mynav_share = get_package_share_directory('mynav')  # 获取原导航包的 share 目录。
    nav2_params_file = os.path.join(mynav_share, 'config', 'nav2_params.yaml')
    behavior_tree_file = os.path.join(mybt_share, 'behavior_trees', 'navigate_to_pose.xml')
    navigation_launch_file = os.path.join(mynav_share, 'launch', 'navigation.launch.py')

    params_file = LaunchConfiguration('params_file')  # 读取完整的 Nav2 参数文件路径。
    bt_xml_file = LaunchConfiguration('bt_xml_file')  # 读取单目标导航行为树文件路径。
    use_sim_time = LaunchConfiguration('use_sim_time')  # 读取是否使用仿真时间。
    autostart = LaunchConfiguration('autostart')  # 读取是否自动激活导航节点。

    declare_params_file = DeclareLaunchArgument(  # 声明 Nav2 参数文件路径。
        'params_file',
        default_value=nav2_params_file,
        description='完整的 Nav2 参数文件',
    )
    declare_bt_xml_file = DeclareLaunchArgument(  # 声明行为树 XML 文件路径。
        'bt_xml_file',
        default_value=behavior_tree_file,
        description='单目标导航行为树 XML 的绝对路径',
    )
    declare_use_sim_time = DeclareLaunchArgument(  # 声明是否使用仿真时间。
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间；真机使用 false',
    )
    declare_autostart = DeclareLaunchArgument(  # 声明是否自动激活生命周期节点。
        'autostart',
        default_value='true',
        description='是否自动激活 Nav2 的生命周期节点',
    )

    # 使用完整参数路径：即使原 YAML 没有这一项，也能正确添加。
    # RewrittenYaml 生成临时文件，不修改 mynav 的原始配置。
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_xml_file,
        },
        convert_types=True,
    )

    nav2_navigation = IncludeLaunchDescription(  # 复用原导航启动文件，启动整套 Nav2。
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'params_file': configured_params,  # 传入已设置默认行为树的参数文件。
            'use_sim_time': use_sim_time,  # 将时钟设置传给导航节点。
            'autostart': autostart,  # 将自动激活设置传给生命周期管理器。
        }.items(),
    )

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_params_file)  # 加入 Nav2 参数文件的启动参数声明。
    ld.add_action(declare_bt_xml_file)  # 加入行为树文件的启动参数声明。
    ld.add_action(declare_use_sim_time)  # 加入仿真时间的启动参数声明。
    ld.add_action(declare_autostart)  # 加入自动激活的启动参数声明。
    ld.add_action(nav2_navigation)  # 加入 Nav2 导航启动动作。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
