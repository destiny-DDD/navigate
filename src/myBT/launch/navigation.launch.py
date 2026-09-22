"""复用 mynav 导航配置，将 myBT 行为树设为单目标导航默认树."""

import os  # 导入路径工具，用来拼接配置文件和行为树文件的路径。

from ament_index_python.packages import get_package_share_directory  # 获取 ROS 2 软件包的安装目录。
from launch import LaunchDescription  # 导入 launch 文件最终需要返回的描述对象。
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # 导入参数声明和包含其他 launch 的动作。
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入 Python launch 文件描述对象。
from launch.substitutions import LaunchConfiguration  # 导入启动参数读取工具。
from nav2_common.launch import RewrittenYaml  # 导入运行时改写 Nav2 YAML 的工具。


def generate_launch_description():  # 定义生成整个 launch 描述的函数。
    mybt_share = get_package_share_directory("my_bt")  # 获取行为树包的 share 目录。
    mynav_share = get_package_share_directory("mynav")  # 获取导航包的 share 目录。
    nav2_params_file = os.path.join(mynav_share, "params", "nav2_params.yaml")  # 拼出 Nav2 参数文件路径。
    behavior_tree_file = os.path.join(mybt_share, "behavior_trees", "navigate_to_pose.xml")  # 拼出行为树文件路径。
    navigation_launch_file = os.path.join(mynav_share, "launch", "mynav2_launch.py")  # 拼出底层 launch 文件路径。

    params_file = LaunchConfiguration("params_file")  # 读取完整的 Nav2 参数文件路径。
    bt_xml_file = LaunchConfiguration("bt_xml_file")  # 读取单目标导航行为树文件路径。
    use_sim_time = LaunchConfiguration("use_sim_time")  # 读取是否使用仿真时间的启动参数。
    autostart = LaunchConfiguration("autostart")  # 读取是否自动激活导航节点的启动参数。

    declare_params_file = DeclareLaunchArgument(  # 声明 Nav2 参数文件路径。
        "params_file",  # 启动参数的名字。
        default_value=nav2_params_file,
        description="完整的 Nav2 参数文件",  # 这个参数在命令行帮助中的说明。
    )
    declare_bt_xml_file = DeclareLaunchArgument(  # 声明行为树 XML 文件路径。
        "bt_xml_file",  # 启动参数的名字。
        default_value=behavior_tree_file,
        description="单目标导航行为树 XML 的绝对路径",  # 这个参数在命令行帮助中的说明。
    )
    declare_use_sim_time = DeclareLaunchArgument(  # 声明是否使用仿真时间。
        "use_sim_time",  # 启动参数的名字。
        default_value="false",  # 真机默认使用系统时间。
        description="是否使用仿真时间；真机使用 false",  # 这个参数在命令行帮助中的说明。
    )
    declare_autostart = DeclareLaunchArgument(  # 声明是否自动激活生命周期节点。
        "autostart",  # 启动参数的名字。
        default_value="true",  # 默认自动激活导航节点。
        description="是否自动激活 Nav2 的生命周期节点",  # 这个参数在命令行帮助中的说明。
    )

    # 使用完整参数路径：即使原 YAML 没有这一项，也能正确添加。
    # RewrittenYaml 生成临时文件，不修改 mynav 的原始配置。
    configured_params = RewrittenYaml(  # 创建改写后的 Nav2 参数文件描述对象。
        source_file=params_file,  # 指定要读取的原始 YAML 文件。
        param_rewrites={  # 指定需要覆盖或新增的参数路径。
            "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml": bt_xml_file,  # 设置单目标导航默认行为树。
        },  # 结束参数改写映射。
        convert_types=True,  # 将启动参数中的字符串转换为合适的参数类型。
    )  # 结束改写后的参数文件定义。

    nav2_navigation = IncludeLaunchDescription(  # 创建复用底层 Nav2 launch 的动作。
        PythonLaunchDescriptionSource(navigation_launch_file),  # 指定底层导航 launch 文件。
        launch_arguments={  # 向底层 launch 传递启动参数。
            "params_file": configured_params,  # 传入已经设置默认行为树的参数文件。
            "use_sim_time": use_sim_time,  # 传递是否使用仿真时间的设置。
            "autostart": autostart,  # 传递是否自动激活生命周期节点的设置。
        }.items(),  # 将参数字典转换为 launch 参数迭代器。
    )  # 结束底层 Nav2 launch 包含动作。

    ld = LaunchDescription()  # 创建一个空的 launch 描述对象。
    ld.add_action(declare_params_file)  # 将 Nav2 参数文件声明加入 launch 描述。
    ld.add_action(declare_bt_xml_file)  # 将行为树文件声明加入 launch 描述。
    ld.add_action(declare_use_sim_time)  # 将仿真时间参数声明加入 launch 描述。
    ld.add_action(declare_autostart)  # 将自动激活参数声明加入 launch 描述。
    ld.add_action(nav2_navigation)  # 将 Nav2 导航启动动作加入 launch 描述。

    return ld  # 返回完整的 launch 描述，供 ROS 2 执行。
