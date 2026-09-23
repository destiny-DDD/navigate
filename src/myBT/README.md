# myBT：调用 Nav2 内置行为树

适用于本项目当前的 **ROS 2 Jazzy**。目录名保留 `myBT`；ROS 包名使用符合规范的小写 `my_bt`，命令使用这个包名。

Nav2 中这个功能叫**行为树（Behavior Tree，BT）**。本包没有自定义 C++ 插件；
`behavior_trees/navigate_to_pose.xml` 采用本机 Nav2 自带的
导航节点组合，由 `bt_navigator` 加载执行，不包含恢复分支。

你现有的 `mynav` 已经启动 Nav2：`explore_lite` 负责挑选探索目标，
`bt_navigator` 负责执行目标的导航行为树。本包提供一个可以查看、修改和指定的树文件。
它负责单目标导航，不包含巡逻、任务调度或自动选择探索目标的逻辑。

## 1. 编译

在工作空间根目录执行（`myBT` 在根目录，colcon 也能发现它）：

```bash
cd /home/future/D/navigate
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mynav my_bt
source install/setup.bash
```

假设原项目依赖已经安装。每个新终端都要 source ROS 环境和工作空间的 `install/setup.bash`。

## 2. 最直接的调用：给已经运行的 Nav2 发一个目标

如果原来的导航栈已经启动，**不用再启动本包的 launch**，直接指定 XML：

```bash
MYBT_XML="$(ros2 pkg prefix --share my_bt)/behavior_trees/navigate_to_pose.xml"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, behavior_tree: '${MYBT_XML}'}" \
  --feedback
```

将 `x`、`y` 换成地图中的可达目标，单位为米；`w: 1.0` 表示朝向 map 坐标系的 +X。
未填写的四元数 x/y/z 默认为 0，未填写的时间戳为 0，使用最新可用变换。
`behavior_tree` 是 **bt_navigator 所在机器**能访问的 XML 绝对路径，不是插件名。
若从另一台电脑发送目标，要填导航主机上的路径。

手动发目标时不要同时运行 `explore_lite`，它会不断发送探索目标，可能抢占你的目标。
若用原项目的整套建图启动入口，使用：

```bash
ros2 launch mynav explore_bringup.launch.py explore:=false
```

该入口已经启动 scan 转换、SLAM 和 Nav2；仍需原项目的传感器、里程计和机器人 TF。
`explore:=false` 关闭自动探索，保留 SLAM 建图。它没有把 myBT 设为默认树，
所以使用上面的 `behavior_tree` 字段来选择本包的树。

## 3. 设为默认树：RViz 和普通导航请求都使用它

本包的 launch **替代** `mynav navigation.launch.py`，只启动一套 Nav2：

```bash
ros2 launch my_bt navigation.launch.py
```

它复用 `mynav/config/nav2_params.yaml`，在启动时设置：

```yaml
bt_navigator:
  ros__parameters:
    default_nav_to_pose_bt_xml: /实际安装目录/share/my_bt/behavior_trees/navigate_to_pose.xml
```

这条 launch 不启动传感器、里程计、机器人 TF、SLAM 或 explore_lite。
沿用原来的底层驱动和机器人 TF；若 scan 转换和 SLAM 尚未启动，可在各自终端运行：

```bash
ros2 launch mynav scan.launch.py
ros2 launch mynav slam.launch.py
```

**不要同时运行第 2 节的 `explore_bringup.launch.py` 和本包 launch**，前者已经包含 Nav2。
导航需要 `/map`、`/scan`、`/odin1/odometry` 以及 `map → odom → base_link` 和传感器 TF。
当前项目用 SLAM 提供地图和 `map → odom`，无需另外启动 AMCL。

Nav2 激活后，RViz 的 **Nav2 Goal** 或下面的请求都会使用默认树：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}, behavior_tree: ''}" \
  --feedback
```

空的 `behavior_tree` 表示使用默认树；非空则只为这次请求选择指定的树。
启动导航不会自动行驶，收到目标后才执行树。
该树针对 `/navigate_to_pose`，不要把它直接用于 `/navigate_through_poses`，后者需要 `{goals}`。

替换默认 XML，或使用仿真时钟：

```bash
ros2 launch my_bt navigation.launch.py bt_xml_file:=/绝对路径/another_tree.xml
ros2 launch my_bt navigation.launch.py use_sim_time:=true
```

仿真模式需要 `/clock`，且相关传感器、SLAM 等节点也要使用一致的时钟。

## 4. 从代码里调用的关键字段

已有 Python `rclpy` 节点中，使用标准 Action 客户端即可（片段需放进你自己的节点）：

```python
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
client.wait_for_server()
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.position.x = 1.0
goal.pose.pose.position.y = 0.0
goal.pose.pose.orientation.w = 1.0
goal.behavior_tree = (
    get_package_share_directory('my_bt') + '/behavior_trees/navigate_to_pose.xml'
)
future = client.send_goal_async(goal)
# 需要 spin 节点处理 future，检查 goal_handle.accepted，
# 再通过 goal_handle.get_result_async() 获取最终成功/失败结果。
# 取消导航使用 goal_handle.cancel_goal_async()，并继续 spin 等待取消完成。
```

使用本包 launch 设置了默认树后，`goal.behavior_tree = ''` 也可以。
不用自己创建 BehaviorTreeFactory，也不用手动 tick 树；这些都由 Nav2 完成。

## 5. 树内如何调用插件

主干是 `ComputePathToPose → FollowPath`：以 1 Hz 重规划，同时持续跟踪路径。
规划或控制失败会直接返回失败，由上层决定是否重新发送目标。

| XML 节点 | 调用的 Nav2 接口/作用 |
| --- | --- |
| `ComputePathToPose` | `/compute_path_to_pose`，默认规划器 ID 为 `GridBased` |
| `FollowPath` | `/follow_path`，默认控制器 ID 为 `FollowPath` |
| `PipelineSequence` | 控制规划与跟踪的执行顺序 |

`{goal}` 由 Nav2 写入黑板；`{path}` 由规划节点输出，供控制节点读取。
`planner_id`、`controller_id` 是 YAML 中配置的 ID，不是 C++ 类名。
本项目已经配置了这些服务器和插件。**Jazzy 自动加载 Nav2 内置 BT 插件**，
无需另外添加 `plugin_lib_names`；自定义插件才需要额外注册。

## 6. 检查是否接入

```bash
ros2 lifecycle get /bt_navigator
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 action list -t
```

导航器应为 `active`；使用本包 launch 时默认 XML 参数应指向 myBT。
仅通过 Action 的 `behavior_tree` 选择树不会修改默认 XML 参数。
导航请求被接受只表示已接收，最终 `SUCCEEDED` 才表示完成。
缺地图、TF、传感器或服务器时，单独存在 XML 不能完成导航。
修改同一路径下的 XML 后，建议重启 Nav2 再验证，避免已加载的树被缓存复用。
