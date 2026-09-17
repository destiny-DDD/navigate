# 自主探索建图 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让机器人在未知环境中自主跑动并持续构建 2D 栅格地图，人可随时喊停。

**Architecture:** MID360 点云经 `pointcloud_to_laserscan` 压成 `/scan`，喂给 `slam_toolbox` 做带回环的 2D SLAM；odin 只提供 `odom`→`imu` 的里程计先验。`slam_toolbox` 出 `/map` 与 `map`→`odom` TF，nav2 在该地图上规划与控制，`explore_lite` 读 nav2 全局 costmap 挑前沿点并通过 `NavigateToPose` 发目标。安全由 `collision_monitor` 独立兜底。

**Tech Stack:** ROS 2 Jazzy · slam_toolbox · nav2 (MPPI + collision_monitor) · explore_lite · pointcloud_to_laserscan · Livox MID360 · odin

**Spec:** `docs/superpowers/specs/2026-09-18-autonomous-exploration-design.md`

## Global Constraints

以下为全项目硬约束，每个任务都隐含包含，不再逐条重复：

- ROS 2 Jazzy，工作区根目录 `/home/shi/nav`，构建用 `colcon build`。
- **禁止出现 `base_footprint`**。URDF 中不存在该 frame；机器人底盘坐标系一律用 `base_link`。
- **里程计话题一律用 `/odin1/odometry`**，绝不写 `/odom`。odin 不发布 `/odom`；`/odom` 是被 `collision_monitor` 输出占用的名字，混用会造成极难排查的反馈环。
- **禁止启动 AMCL 与 `map_server`**。AMCL 的 `tf_broadcast` 默认为 true，会与 `slam_toolbox` 争夺 `map`→`odom` 发布权。因此只用 `nav2_bringup/launch/navigation_launch.py`，不用 `bringup_launch.py`。
  - 注意 `bringup_launch.py` **并非没有 slam**：它按真值表二选一 —— `slam and use_localization` → `slam_launch.py`（:156-160），`not slam and use_localization` → `localization_launch.py`（:169-173），而 `slam` 默认 `False`（:95-96）。
  - 不用它的真实原因是另外两条：(1) `slam_launch.py:114-140` 会把 `params_file` 整份转交给 slam_toolbox，要求 slam 参数并入 `nav2_params.yaml`，而我们两者是独立文件；(2) `slam_launch.py:44` 用的是 `online_sync`，我们主动选了 `online_async`。
- 底盘为**四轮麦克纳姆轮**（URDF `wheel_1..4`，位于 `(±0.11, ±0.1605, 0.0762)`），全向。motion_model 一律 `Omni`。
- 真机运行，**`use_sim_time` 一律显式设 `false`**。真机无 `/clock`，保持 true 会让节点静默空转。
- `explore_lite` 的参数文件根键必须是 `explore_node:`，且 launch 中 `Node(name="explore_node")`，两者必须一致，否则参数静默不生效。
- 所有新增 YAML 配置的中文注释风格与现有 `src/mynav/config/nav2_params.yaml` 保持一致（每个键一行行尾注释）。
- **用户 2026-09-18 明确要求暂不提交 git**（针对本轮设计文档）。本计划各任务末尾保留 commit 步骤以维持方法论完整；执行时先跳过，若要改为提交需先向用户确认。

## 与 Spec 的两处偏离（已确认合理，执行时按本计划）

1. Spec §6 的 `launch/navigation.launch.py` 写作"include nav2 navigation_launch.py + explore_lite"。本计划拆为 `navigation.launch.py`（仅 nav2）与 `explore.launch.py`（仅 explore_lite），再由 `explore_bringup.launch.py` 用 `explore:=true|false` 条件合并。原因：Spec §8 第 4 步要求"接入探索之前必须单独验证 nav2 单点导航"，打包在一起就无法关闭探索做该验证。
2. Spec §6 列出的 `rviz/explore.rviz` 本期不做。`mysystem` 已启动 `foxglove_bridge`（端口 8765），验证可视化统一走 Foxglove，RViz 配置属冗余。

---

### Task 1: 屏蔽 map_merge，搭建 myexplore 包骨架

**Files:**
- Create: `src/m-explore-ros2/map_merge/COLCON_IGNORE`（空文件）
- Move: `myexplore/`（仓库根目录，用户已创建的模板空壳）→ `src/myexplore/`
- Rewrite: `src/myexplore/package.xml`
- Rewrite: `src/myexplore/CMakeLists.txt`
- Delete: `src/myexplore/src/`、`src/myexplore/include/`（模板生成的空目录，纯配置包用不上）
- Create: `src/myexplore/launch/.gitkeep`
- Create: `src/myexplore/config/.gitkeep`

**Interfaces:**
- Consumes: 无（首个任务）
- Produces: 包名 `myexplore`，其 share 目录下的 `launch/` 与 `config/` 供后续所有任务放置文件。后续任务一律通过 `get_package_share_directory("myexplore")` 定位。

- [x] **Step 1: 屏蔽 map_merge 包**

`multirobot_map_merge` 是多机地图融合，单机不需要，且其 README 自述依赖 ROS1 的 `slam_gmapping`（从未移植到 ROS2）。它在编译期会下载测试地图并因本地代理 SSL 断开而产生失败噪音，耗时约 49 秒。

```bash
touch src/m-explore-ros2/map_merge/COLCON_IGNORE
```

- [x] **Step 2: 清掉模板留下的空目录**

> 原步骤"把根目录 `myexplore/` 挪进 `src/`"已由用户于 2026-09-18 自行完成（`ros2 pkg create` 生成的空壳现已在 `src/myexplore/`），故此处只剩清理。

```bash
# 纯配置包没有编译目标，用不上这两个空目录（留着会让后来者以为这里要写 C++）。
rmdir src/myexplore/src src/myexplore/include/myexplore src/myexplore/include
```

清理后确认：

```bash
find src/myexplore | sort
```

Expected:
```
src/myexplore
src/myexplore/CMakeLists.txt
src/myexplore/LICENSE
src/myexplore/package.xml
```
（`LICENSE` 保留，与 `src/mynav` 的做法一致）

- [x] **Step 3: 覆盖 package.xml**

`mv` 过来的 `package.xml` 是模板生成的无用内容（`<description>TODO: Package description</description>`，且依赖只有 `rclcpp`）。整体替换为：

创建 `src/myexplore/package.xml`：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>myexplore</name>
  <version>0.0.0</version>
  <description>自主探索建图：MID360 点云转 2D 扫描，slam_toolbox 建图，nav2 导航，explore_lite 决策。</description>
  <maintainer email="shifengddd@gmail.com">shi</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 本包只安装 launch 与 config，无编译目标，因此依赖全部是 exec_depend。 -->
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_mppi_controller</exec_depend>
  <exec_depend>nav2_collision_monitor</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>pointcloud_to_laserscan</exec_depend>
  <exec_depend>explore_lite</exec_depend>
  <exec_depend>livox_ros_driver2</exec_depend>
  <exec_depend>odin_ros_driver</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [x] **Step 4: 覆盖 CMakeLists.txt**

同样把模板生成的内容整体替换为：

创建 `src/myexplore/CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.8)
project(myexplore)

find_package(ament_cmake REQUIRED)

# 本包无编译目标，只把 launch 与 config 装进 share/，供 ros2 launch 读取。
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 跳过版权检查与 cpplint：本包是纯配置包，且不在 git 仓库的常规位置。
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

- [x] **Step 5: 建空的 launch/ 与 config/ 目录**

`install(DIRECTORY ...)` 在目录不存在时会直接报错，所以先占位。

```bash
mkdir -p src/myexplore/launch src/myexplore/config
touch src/myexplore/launch/.gitkeep src/myexplore/config/.gitkeep
```

- [x] **Step 6: 构建并验证**

```bash
colcon build --packages-select myexplore
```

Expected: `Summary: 1 package finished [N s]`，无 stderr 报错。

```bash
source install/setup.bash
ros2 pkg prefix myexplore
```

Expected: `/home/shi/nav/install/myexplore`

```bash
colcon list | grep -c map_merge
```

Expected: `0`（屏蔽生效；此前 `multirobot_map_merge` 会出现在列表中）

- [x] **Step 7: Commit**

```bash
git add src/myexplore src/m-explore-ros2/map_merge/COLCON_IGNORE
git commit -m "feat(myexplore): 新建探索包骨架并屏蔽 map_merge"
```

---

### Task 2: Livox 改发 PointCloud2 + 3D 点云转 2D 扫描

**Files:**
- Modify: `src/mycontrol/launch/mycontrol_launch.py`（`xfer_format` 1 → 0）
- Create: `src/myexplore/config/scan.yaml`
- Create: `src/myexplore/launch/scan.launch.py`

**Interfaces:**
- Consumes: `myexplore` 包（Task 1）
- Produces: 话题 `/scan`（`sensor_msgs/msg/LaserScan`），`frame_id` 为 `base_link`，供 Task 3 的 `slam_toolbox` 与 Task 4/5 的 nav2 costmap 消费。

**背景（务必先读）**：现有 `mycontrol_launch.py` 设 `{"xfer_format": 1}`。`src/livox_ros_driver2/src/lddc.h:42-47` 定义 `kPointCloud2Msg = 0, kLivoxCustomMsg = 1`，即 **1 发的是 `livox_ros_driver2/msg/CustomMsg`**。`pointcloud_to_laserscan` 只接受 `sensor_msgs/msg/PointCloud2`，不改这一项整条链路直接断。

- [x] **Step 1: 修正 Livox 输出格式**

在 `src/mycontrol/launch/mycontrol_launch.py` 中，把 livox 节点参数里的 `xfer_format` 改掉：

```python
            {"xfer_format": 1},  # 1 表示使用 Livox 自定义点云格式，0 表示标准 PointCloud2。
```

改为：

```python
            # 必须是 0（kPointCloud2Msg）。1 是 livox_ros_driver2/msg/CustomMsg，
            # 下游 pointcloud_to_laserscan 只吃 sensor_msgs/PointCloud2，填 1 会直接断链。
            {"xfer_format": 0},
```

- [x] **Step 2: 写 scan.yaml**

创建 `src/myexplore/config/scan.yaml`：

```yaml
pointcloud_to_laserscan:
  ros__parameters:
    target_frame: base_link        # 先变换到机体坐标系再做高度切片，使高度窗口有明确物理含义。
    transform_tolerance: 0.01      # 点云变换到 target_frame 允许的 TF 时间误差，单位秒。
    min_height: -0.15              # 高度窗口下界（相对 base_link），起始值供调参，非最终值。
    max_height: 0.25               # 高度窗口上界（相对 base_link），起始值供调参，非最终值。
    angle_min: -3.14159265         # 扫描起始角，-π 表示全向覆盖。
    angle_max: 3.14159265          # 扫描结束角，π 表示全向覆盖。
    angle_increment: 0.0087        # 相邻扫描束的角度间隔，约 0.5 度，360 度共约 720 束。
    scan_time: 0.1                 # 对应 MID360 的 10 Hz 发布频率，单位秒。
    range_min: 0.2                 # 小于此距离的回波丢弃，避免机体自身被标为障碍，单位米。
    range_max: 20.0                # 最大有效距离，与 slam_toolbox 的 max_laser_range 对齐，单位米。
    use_inf: true                  # 无回波方向输出 inf 而非直接丢弃，让建图知道该方向是空的。
    inf_epsilon: 1.0               # 判定为无穷远的容差，单位米。
    queue_size: 30                 # 订阅队列长度，按 10 Hz 计约可缓冲 3 秒。
```

**不要加 `concurrency_level`**。那是 ROS1 版遗留的参数，ROS2 的 `pointcloud_to_laserscan_node` 并未声明它 —— 实测 `ros2 param list /pointcloud_to_laserscan` 只有上表列出的那些。ROS 2 对未声明参数是**静默忽略**（不报错），所以填了不会有任何症状，只会让人误以为配了单线程回调。这正是本项目里 `local_costmap.static_layer` 那类死配置的翻版。

上表全部参数名已实测核对过。唯一会自动存在但无需填写的是 `use_sim_time`（由 `rclcpp::Node` 自动声明）。

- [x] **Step 3: 写 scan.launch.py**

创建 `src/myexplore/launch/scan.launch.py`：

```python
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
```

`use_sim_time` 显式写 `False`（`rclcpp::Node` 自动声明该参数，可直接传）。真机无 `/clock`，全局约束要求所有节点显式设 false。

- [x] **Step 4: 构建并做语法检查**

```bash
python3 -m py_compile src/myexplore/launch/scan.launch.py && echo "launch 语法 OK"
python3 -c "import yaml,sys; yaml.safe_load(open('src/myexplore/config/scan.yaml')); print('yaml 语法 OK')"
colcon build --packages-select myexplore mycontrol
```

Expected: 三行都成功，`Summary: 2 packages finished`。

- [ ] **Step 5: 确认点云格式已变**

先起底层（终端 A）：

```bash
./mystart.sh
```

若尚未改 `mystart.sh`（Task 7 才改），则手动分别起：

```bash
source install/setup.bash
ros2 launch mysystem mysystem_love_launch.py &
ros2 launch mycontrol mycontrol_launch.py
```

另开终端 B 确认话题类型（**这是本任务的关键验证点**）：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic info /livox/lidar --verbose | grep -i "type\|Type"
```

Expected: `Type: sensor_msgs/msg/PointCloud2`

若显示 `livox_ros_driver2/msg/CustomMsg`，说明 Step 1 没生效或没重新 build，**不要继续**，回到 Step 1。

```bash
ros2 topic hz /livox/lidar
```

Expected: `average rate: 10.0` 左右（±0.5）。

- [ ] **Step 6: 起 scan 层并验证 /scan**

终端 B 继续：

```bash
ros2 launch myexplore scan.launch.py
```

另开终端 C：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic hz /scan
```

Expected: `average rate: 10.0` 左右。

逐项核对关键字段（整条 `/scan` 太大，不要直接 `echo`，用 `--field` 取单字段）：

```bash
ros2 topic echo /scan --field header.frame_id --once   # 期望 base_link
ros2 topic echo /scan --field range_max --once         # 期望 20.0
ros2 topic echo /scan --field angle_increment --once   # 期望约 0.0087
```

统计有效回波数（排除 `inf` 与 `nan`）：

```bash
ros2 topic echo /scan --field ranges --once | tr -d '[]' | tr ',' '\n' | grep -c -E '^[[:space:]]*-?[0-9]'
```

Expected: 大于 100。若为 0 或个位数，说明高度窗口没切到有效点云，进 Step 7。

- [ ] **Step 7: 确定雷达实际安装角并分支处理**

`robot.urdf` 中 `joint_livox_frame` 的 origin 带 `rpy="0.00000735 0.78539265 3.14159265"`，即相对 `up` 有约 45° 俯仰。这直接决定高度切片是否有效，必须先实测确认。

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

观察输出的 `- Rotation: in RPY (radian)` 一行：

**分支 A —— pitch 接近 0**：URDF 与实际安装不符，实际雷达是水平装的。此时修改 `src/mysystem/urdf/robot.urdf` 中 `joint_livox_frame` 的 `rpy`，把中间的 pitch 分量改为 0，保留 yaw。改完重启 `mysystem`。这是正确修法，不要靠调高度窗口去凑。

**分支 B —— pitch 确实约 ±0.785**：雷达真的斜装了。此时高度切片切的是锥面而非水平面，先按放宽窗口让它跑通：

```yaml
    min_height: -1.0               # 雷达斜装，放宽窗口先让链路跑通。
    max_height: 1.0                # 同上。
```

改完重跑 Step 6。并把"把 MID360 改水平安装"记入硬件待办——分支 B 下 `/scan` 的有效距离会明显短于水平安装，长期必须改硬件。

- [ ] **Step 8: 在 Foxglove 中确认扫描质量**

连 Foxglove 到 `ws://<车机IP>:8765`，添加 3D 面板，显示 `/scan`（设为 Points）与 `/livox/lidar`（PointCloud2）叠加。

缓慢原地转动车体，确认：

- 扫描点连成完整一圈，无大片缺口
- 扫描点落在与雷达同高的水平面上，不随转动上下摆动（摆动即为分支 B）
- 地面与天花板未被误标（若被标，收紧 `min_height`/`max_height`）
- 用卷尺量一面墙，读数与实际相符

**此步不通过则后续全部无意义** —— `slam_toolbox` 的建图质量完全取决于 `/scan`。

- [ ] **Step 9: Commit**

```bash
git add src/mycontrol/launch/mycontrol_launch.py src/myexplore/config/scan.yaml src/myexplore/launch/scan.launch.py
git commit -m "feat(myexplore): MID360 点云转 2D 扫描，Livox 改发 PointCloud2"
```

---

### Task 3: slam_toolbox 建图层

**Files:**
- Create: `src/myexplore/config/slam_toolbox.yaml`
- Create: `src/myexplore/launch/slam.launch.py`

**Interfaces:**
- Consumes: `/scan`（Task 2）、`odom`→`imu` TF 与 `/odin1/odometry`（odin，已就绪）
- Produces: `/map`（`nav_msgs/msg/OccupancyGrid`）、`map`→`odom` TF。Task 4/5 的 nav2 costmap 与 Task 6 的 explore_lite 都消费这两个。

- [x] **Step 1: 写 slam_toolbox.yaml**

以 `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml` 为基准，**只改 `base_frame` 一项**。

```bash
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml \
   src/myexplore/config/slam_toolbox.yaml
```

然后修改 `src/myexplore/config/slam_toolbox.yaml` 这一行：

```yaml
    base_frame: base_footprint
```

改为：

```yaml
    base_frame: base_link
```

**不改 `odom_frame: odom`** —— odin 发布的正是 `odom`→`imu`，与 URDF 的 `imu`→…→`base_link` 链接起来后，整条 `odom`→`base_link` 通路成立。

其余与本期相关的默认值保持不动，确认它们存在即可：`mode: mapping`、`do_loop_closing: true`、`scan_topic: /scan`、`use_scan_matching: true`、`resolution: 0.05`、`transform_publish_period: 0.02`、`max_laser_range: 20.0`。

- [x] **Step 2: 写 slam.launch.py**

创建 `src/myexplore/launch/slam.launch.py`：

```python
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
```

`use_sim_time` 必须显式传 `false`：上游 `online_async_launch.py:32` 的默认值是 `'true'`。

- [x] **Step 3: 构建并做语法检查**

```bash
python3 -m py_compile src/myexplore/launch/slam.launch.py && echo "launch 语法 OK"
python3 -c "import yaml; yaml.safe_load(open('src/myexplore/config/slam_toolbox.yaml')); print('yaml 语法 OK')"
colcon build --packages-select myexplore
```

Expected: 全部成功。

- [ ] **Step 4: 启动并确认节点已激活**

保持 Task 2 的 `mysystem` + `mycontrol` + `scan.launch.py` 运行。新终端：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch myexplore slam.launch.py
```

Expected 日志：`[LifecycleLaunch] Slamtoolbox node is activating.`，随后 `slam_toolbox` 报 `Using solver plugin solver_plugins::CeresSolver` 与 `SlamToolbox: Starting...`。

**若卡在 inactive**：检查 `ros2 lifecycle get /slam_toolbox`。停在 `inactive` 说明 `autostart` 没生效。

- [ ] **Step 5: 验证 TF 树**

新终端：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run tf2_tools view_frames
```

打开生成的 `frames.pdf`（或看终端输出），确认链路完整：

```
map ──▶ odom ──▶ imu ──▶ up ──┬──▶ livox_frame
                              └──▶ base_link ──▶ wheel_1..4
```

**关键检查：每个 frame 只有一个父节点。** 若 `map` 或 `odom` 出现两个父节点（或报 "TF tree is not connected"），说明有 AMCL 在跑，执行 `ros2 node list | grep amcl` 确认并杀掉。

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Expected: 持续输出变换，`Translation` 不跳变。

- [ ] **Step 6: 验证建图与回环**

```bash
ros2 topic hz /map
```

Expected: 约 0.2 Hz（`map_update_interval: 5.0`）。

**手动遥控车走一圈回到起点**（用 Foxglove 的 teleop，或 `ros2 run teleop_twist_keyboard teleop_twist_keyboard`），然后：

```bash
ros2 topic echo /map --field info --once
```

Expected: `width`/`height` 随行进扩大。

在 Foxglove 中添加 Map 面板显示 `/map`，确认：

- 地图边界与实际房间轮廓吻合
- **回到起点后，同一面墙只有一层。出现双层墙即回环未生效**，需检查 `do_loop_closing` 是否为 true、`minimum_travel_distance: 0.5` 是否过大（走得太慢时回环触发不了，可调到 0.3）

- [ ] **Step 7: Commit**

```bash
git add src/myexplore/config/slam_toolbox.yaml src/myexplore/launch/slam.launch.py
git commit -m "feat(myexplore): 接入 slam_toolbox 在线异步建图"
```

---

### Task 4: nav2 参数迁移与修正 + navigation.launch.py

**Files:**
- Create: `src/myexplore/config/nav2_params.yaml`（自 `src/mynav/config/nav2_params.yaml` 迁移）
- Create: `src/myexplore/launch/navigation.launch.py`
- Create: `tools/check_nav2_params.py`（自动断言脚本）

**Interfaces:**
- Consumes: `/scan`（Task 2）、`/map` 与 `map`→`odom` TF（Task 3）、`/odin1/odometry`（odin）
- Produces: nav2 各生命周期节点、`NavigateToPose` action server（Task 6 的 explore_lite 消费）、话题 `/cmd_vel_nav` → `/cmd_vel_smoothed` → `/cmd_vel`（Task 5 的 collision_monitor 与 Task 6 后续链路消费）

- [x] **Step 1: 迁移参数文件**

```bash
cp src/mynav/config/nav2_params.yaml src/myexplore/config/nav2_params.yaml
```

- [x] **Step 2: 写自动断言脚本**

这是本任务的可执行测试：它把"哪些值必须是修正后的"变成一条命令可验证的断言。

创建 `tools/check_nav2_params.py`：

```python
#!/usr/bin/env python3
"""断言 nav2_params.yaml 中麦轮全向与坐标系相关的关键项已修正。

用法: python3 tools/check_nav2_params.py [path]
退出码 0 表示全部通过，1 表示有未通过项。
"""
import re
import sys

import yaml

path = sys.argv[1] if len(sys.argv) > 1 else "src/myexplore/config/nav2_params.yaml"

with open(path) as f:
    raw = f.read()
p = yaml.safe_load(raw)

FOOTPRINT = "[[-0.19, -0.24], [-0.19, 0.24], [0.19, 0.24], [0.19, -0.24]]"

LC = ("local_costmap", "local_costmap", "ros__parameters")
GC = ("global_costmap", "global_costmap", "ros__parameters")
CS = ("controller_server", "ros__parameters")
FP = CS + ("FollowPath",)
VS = ("velocity_smoother", "ros__parameters")
CM = ("collision_monitor", "ros__parameters")

# (说明, 取值路径, 期望值)
PATH_CHECKS = [
    # --- 麦轮全向：前三条漏改会导致横移能力完全失效 ---
    ("FollowPath.motion_model 为 Omni", FP + ("motion_model",), "Omni"),
    ("min_y_velocity_threshold 已放开", CS + ("min_y_velocity_threshold",), 0.1),
    ("velocity_smoother.max_velocity 的 Y 非 0", VS + ("max_velocity",), [0.5, 0.5, 2.0]),
    ("velocity_smoother.min_velocity 的 Y 非 0", VS + ("min_velocity",), [-0.5, -0.5, -2.0]),
    ("velocity_smoother.max_accel 的 Y 非 0", VS + ("max_accel",), [2.5, 2.5, 3.2]),
    ("velocity_smoother.max_decel 的 Y 非 0", VS + ("max_decel",), [-2.5, -2.5, -3.2]),
    # --- 里程计话题 ---
    ("bt_navigator.odom_topic", ("bt_navigator", "ros__parameters", "odom_topic"), "/odin1/odometry"),
    ("velocity_smoother.odom_topic", VS + ("odom_topic",), "/odin1/odometry"),
    # --- 坐标系 ---
    ("collision_monitor.base_frame_id", CM + ("base_frame_id",), "base_link"),
    ("global_costmap.robot_base_frame", GC + ("robot_base_frame",), "base_link"),
    ("local_costmap.robot_base_frame", LC + ("robot_base_frame",), "base_link"),
    ("amcl.base_frame_id", ("amcl", "ros__parameters", "base_frame_id"), "base_link"),
    ("loopback_simulator.base_frame_id",
     ("loopback_simulator", "ros__parameters", "base_frame_id"), "base_link"),
    # --- 机器人轮廓：实测包围盒 x±0.186 / y±0.237，留 5mm 余量 ---
    ("global_costmap.footprint", GC + ("footprint",), FOOTPRINT),
    ("local_costmap.footprint", LC + ("footprint",), FOOTPRINT),
    ("global_costmap 已移除 robot_radius", GC + ("robot_radius",), None),
    ("local_costmap 已移除 robot_radius", LC + ("robot_radius",), None),
    # --- MPPI 按真实轮廓做碰撞检查（原为 false，配合偏小的 robot_radius 会漏检）---
    ("CostCritic.consider_footprint", FP + ("CostCritic", "consider_footprint"), True),
    # --- local_costmap 的死配置已清除 ---
    ("local_costmap.plugins", LC + ("plugins",), ["voxel_layer", "inflation_layer"]),
    ("local_costmap 的 static_layer 段已删除", LC + ("static_layer",), None),
    # --- 探索必需项，保持不变 ---
    ("planner_server.GridBased.allow_unknown",
     ("planner_server", "ros__parameters", "GridBased", "allow_unknown"), True),
    ("global_costmap.plugins 保留 static_layer",
     GC + ("plugins",), ["static_layer", "obstacle_layer", "inflation_layer"]),
]

# (说明, 应出现的子串, 期望是否出现)
RAW_CHECKS = [
    ("全文不含 base_footprint", "base_footprint", False),
    ("全文不含 robot_radius", "robot_radius", False),
]

# 剥离行尾注释后再做子串检查：注释里提到某个名字（例如解释"为何不用 base_footprint"）
# 不代表它是个生效的配置值，用原文匹配会误报。
uncommented = "\n".join(line.split("#", 1)[0] for line in raw.splitlines())

# 取值可能是 /odom、odom、"/odom"、'/odom' 等多种写法，子串匹配会漏，
# 因此把每一处 odom_topic 的取值都抽出来，要求它们全部指向 odin。
# 这条检查的价值在于能发现 PATH_CHECKS 里没列到的其他 odom_topic 出现处。
odom_sites = sorted(set(re.findall(r"^\s*odom_topic:\s*[\"']?([^\"'\s#]+)", raw, re.M)))
COMPUTED_CHECKS = [
    ("所有 odom_topic 均指向 /odin1/odometry", odom_sites, ["/odin1/odometry"]),
]


def dig(obj, keys):
    """按路径取值；中途缺失返回 None，以便一次性看全所有失败项而非中途崩溃。"""
    for k in keys:
        if not isinstance(obj, dict) or k not in obj:
            return None
        obj = obj[k]
    return obj


failed = 0
total = 0


def report(name, got, want):
    global failed, total
    total += 1
    if got != want:
        failed += 1
        print(f"FAIL  {name}  got={got!r} want={want!r}")
    else:
        print(f"PASS  {name}")


for name, keys, want in PATH_CHECKS:
    report(name, dig(p, keys), want)

for name, needle, want in RAW_CHECKS:
    report(name, needle in uncommented, want)

for name, got, want in COMPUTED_CHECKS:
    report(name, got, want)

print(f"\n{total - failed}/{total} 通过")
sys.exit(1 if failed else 0)
```

- [x] **Step 3: 运行断言脚本，确认它失败**

```bash
python3 tools/check_nav2_params.py
```

Expected: 大量 `FAIL`，末行 `5/25 通过`，退出码 1。此时文件还是原样迁移过来的未修正版本。（5 项通过的是本就不需要改的：两处 `robot_base_frame: base_link`、`local_costmap.plugins`、`planner_server.GridBased.allow_unknown`、`global_costmap.plugins`。）

```bash
echo "退出码: $?"
```

Expected: `退出码: 1`

- [x] **Step 4: 修正 bt_navigator 的里程计话题**

在 `src/myexplore/config/nav2_params.yaml` 中：

```yaml
    odom_topic: /odom  # 里程计话题，用于行为树获取机器人运动状态。
```

改为：

```yaml
    odom_topic: /odin1/odometry  # 里程计话题；odin 发的是 /odin1/odometry，不是 /odom。
```

- [x] **Step 5: 修正 controller_server 的横向速度阈值**

```yaml
    min_y_velocity_threshold: 0.5  # 忽略小于此值的横向速度，差速车通常不使用横移。
```

改为：

```yaml
    min_y_velocity_threshold: 0.1  # 忽略小于此值的横向速度；原值 0.5 是差速车设置，会滤掉麦轮横移。
```

- [x] **Step 6: 修正 MPPI 运动模型与碰撞检查**

```yaml
      motion_model: "DiffDrive"  # 使用差速驱动运动模型。
```

改为：

```yaml
      motion_model: "Omni"  # 麦轮全向底盘，必须用 Omni；填 DiffDrive 会完全丧失横移能力。
```

```yaml
        consider_footprint: false  # 是否用完整机器人轮廓而不是中心点检查碰撞。
```

改为：

```yaml
        consider_footprint: true  # 按真实轮廓检查碰撞；本车 0.37×0.47 m，按中心点检查会漏检。
```

- [x] **Step 7: 删除 local_costmap 的死配置**

`local_costmap` 的 `static_layer` 段存在但不在 `plugins` 列表中，是无效配置，且 `StaticLayer` 与 `rolling_window: true` 组合本身也不成立（滚动窗口没有固定静态地图）。删除整段：

```yaml
      static_layer:  # 局部地图静态地图层配置段。
        plugin: "nav2_costmap_2d::StaticLayer"  # 从静态地图服务器读取地图的图层。
        map_subscribe_transient_local: True  # 使用瞬态本地订阅，确保能收到已发布的地图。
```

- [x] **Step 8: 修正机器人轮廓**

实测（由 `src/mysystem/meshes/*.stl` 包围盒叠加关节 origin 得出）：机体整体 x ∈ [-0.1862, +0.1862]，y ∈ [-0.2367, +0.2367]，最远角点距原点 0.301 m。原 `robot_radius: 0.22` **偏小 27%**，会导致碰撞检查漏检。

`local_costmap` 与 `global_costmap` 两处都改。删除各自的 `robot_radius` 行：

```yaml
      robot_radius: 0.22  # 机器人碰撞半径，单位米。
```

替换为：

```yaml
      # 用多边形轮廓而非圆半径：实测包围盒 x±0.186、y±0.237（外扩 5mm），
      # 圆半径需 0.30 才覆盖得住，但多边形更贴合，窄通道通过性更好。
      footprint: "[[-0.19, -0.24], [-0.19, 0.24], [0.19, 0.24], [0.19, -0.24]]"
```

`global_costmap` 中该行的注释文案相同，同样处理。

- [x] **Step 9: 修正 velocity_smoother 的横向分量与里程计话题**

```yaml
    max_velocity: [0.5, 0.0, 2.0]  # 最大速度 [前进、横向、旋转]，单位分别为米/秒、米/秒、弧度/秒。
    min_velocity: [-0.5, 0.0, -2.0]  # 最小速度 [前进、横向、旋转]。
    max_accel: [2.5, 0.0, 3.2]  # 最大加速度 [前进、横向、旋转]。
    max_decel: [-2.5, 0.0, -3.2]  # 最大减速度 [前进、横向、旋转]。
    odom_topic: "odom"  # 用于速度反馈的里程计话题。
```

改为：

```yaml
    max_velocity: [0.5, 0.5, 2.0]  # 最大速度 [前进、横向、旋转]；Y 原为 0，会把横移清零。
    min_velocity: [-0.5, -0.5, -2.0]  # 最小速度 [前进、横向、旋转]；Y 原为 0，同上。
    max_accel: [2.5, 2.5, 3.2]  # 最大加速度 [前进、横向、旋转]；Y 原为 0，同上。
    max_decel: [-2.5, -2.5, -3.2]  # 最大减速度 [前进、横向、旋转]；Y 原为 0，同上。
    odom_topic: "/odin1/odometry"  # 用于速度反馈的里程计话题；odin 不发 /odom。
```

- [x] **Step 10: 修正 collision_monitor 与 amcl 的坐标系**

```yaml
    base_frame_id: "base_footprint"  # 碰撞监控使用的机器人底盘坐标系。
```

改为：

```yaml
    base_frame_id: "base_link"  # 机器人底盘坐标系；URDF 中不存在 base_footprint，填错节点起不来。
```

```yaml
    base_frame_id: "base_footprint"  # 机器人底盘坐标系名称。
```

改为：

```yaml
    base_frame_id: "base_link"  # 机器人底盘坐标系名称；本配置不启动 AMCL，此项仅保持一致避免误用。
```

```yaml
    base_frame_id: "base_footprint"  # 机器人底盘坐标系。
```

改为：

```yaml
    base_frame_id: "base_link"  # 机器人底盘坐标系。
```

- [x] **Step 11: 在 amcl 段加警示注释**

`amcl` 段在本配置中是死配置（`navigation_launch.py` 不启动 AMCL），但留着容易被后来者误当成"已配置好定位"。在 `amcl:` 段标题上方插入：

```yaml
# 注意：本配置使用 slam_toolbox 建图，不启动 AMCL 与 map_server。
# 下面这段 amcl 参数不会生效，保留仅为将来切换定位模式时参考。
# 切勿在启动 slam_toolbox 的同时启动 AMCL —— 两者都会发布 map->odom，会造成 TF 冲突。
amcl:  # AMCL 粒子滤波定位节点。
```

- [x] **Step 12: 运行断言脚本，确认全部通过**

```bash
python3 tools/check_nav2_params.py
```

Expected: 全部 `PASS`，末行 `25/25 通过`，退出码 0。

若有 `FAIL`，按提示回到对应 Step 修正。

- [x] **Step 13: 写 navigation.launch.py**

创建 `src/myexplore/launch/navigation.launch.py`：

```python
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
```

- [x] **Step 14: 构建并检查启动参数**

```bash
python3 -m py_compile src/myexplore/launch/navigation.launch.py && echo "launch 语法 OK"
colcon build --packages-select myexplore
source install/setup.bash
ros2 launch myexplore navigation.launch.py --show-args
```

Expected: 列出 `params_file` / `use_sim_time` / `autostart` 三个参数，`use_sim_time` 默认 `'false'`。

- [ ] **Step 15: 启动并确认生命周期节点全部激活**

保持 `mysystem` + `mycontrol` + `scan.launch.py` + `slam.launch.py` 运行。新终端：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch myexplore navigation.launch.py
```

另开终端确认生命周期状态：

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /collision_monitor
```

Expected: 全部 `active`。

**确认 AMCL 没被启动**：

```bash
ros2 node list | grep -i "amcl\|map_server"
```

Expected: 无输出。

```bash
ros2 topic info /cmd_vel_smoothed
```

Expected: 有发布者（velocity_smoother）与订阅者（collision_monitor）。

- [ ] **Step 16: 验证单点导航**

在 Foxglove 中：

1. 用 "2D Pose Estimate" 设定初始位姿（此步仅影响显示；`slam_toolbox` 的 map 原点即启动位置）
2. 用 "2D Goal Pose" 在已建图区域内发一个 1~2 米外的目标

Expected: `planner_server` 报出路径（`Received a goal, begin computing control effort` / `Found path`），`controller_server` 驱动车体移动并到达，机器人停下。

**在到达前先测横移**：发一个目标在机器人正侧方（正左或正右 1 米），观察车是否直接横向平移过去而非先转向。若只转不前，说明 `motion_model` 或 Y 速度限幅仍有问题，回到 `python3 tools/check_nav2_params.py`。

**若规划失败**：检查 `ros2 topic echo /global_costmap/costmap --field info --once` 是否有非零尺寸；检查 `ros2 topic hz /scan` 是否仍在 10 Hz。

**这一步必须在接入探索之前单独验证通过**，否则无法区分后续故障来自导航还是探索。

- [ ] **Step 17: Commit**

```bash
git add src/myexplore/config/nav2_params.yaml src/myexplore/launch/navigation.launch.py tools/check_nav2_params.py
git commit -m "feat(myexplore): 迁移 nav2 参数并修正为麦轮全向构型"
```

---

### Task 5: collision_monitor 安全区

**Files:**
- Modify: `src/myexplore/config/nav2_params.yaml`（`collision_monitor` 段）

**Interfaces:**
- Consumes: Task 4 的 `collision_monitor` 节点与 `nav2_params.yaml`
- Produces: 话题 `/cmd_vel`（`geometry_msgs/msg/Twist`），由 `mycontrol` 订阅。Task 6 的自主运动全部经过这一关。

**为什么这层不是可选项**：需求是"一直探索"，机器人在无人看管下自主移动。`collision_monitor` 直接拦在 `cmd_vel_smoothed` 与最终 `cmd_vel` 之间，是独立于探索逻辑的最后一关——即使 `explore_lite` 或 nav2 行为异常，它也会拦下。

- [x] **Step 1: 写自动断言脚本的扩充项**

在 `tools/check_nav2_params.py` 的 `PATH_CHECKS` 列表末尾（`global_costmap.plugins 保留 static_layer` 那组之后、`]` 之前）插入：

```python
    # --- collision_monitor 安全区 ---
    ("collision_monitor.polygons", CM + ("polygons",), ["StopZone", "SlowZone", "FootprintApproach"]),
    ("StopZone.action_type 为 stop", CM + ("StopZone", "action_type"), "stop"),
    ("SlowZone.action_type 为 slowdown", CM + ("SlowZone", "action_type"), "slowdown"),
    ("stop_pub_timeout 放宽到 5 秒", CM + ("stop_pub_timeout",), 5.0),
```

`dig()` 对缺失的中间键返回 `None`，所以 `StopZone` 尚未定义时这几项会正常报 `FAIL` 而不是崩溃。

- [x] **Step 2: 运行断言脚本，确认新增项失败**

```bash
python3 tools/check_nav2_params.py
```

Expected: 前 25 项全 `PASS`，新增的 4 项全 `FAIL`，末行 `25/29 通过`。

- [x] **Step 3: 扩充 polygons 列表**

在 `src/myexplore/config/nav2_params.yaml` 中：

```yaml
    polygons: ["FootprintApproach"]  # 启用的碰撞区域名称。
```

改为：

```yaml
    polygons: ["StopZone", "SlowZone", "FootprintApproach"]  # 启用的碰撞区域名称。
```

- [x] **Step 4: 定义急停区与减速区**

紧接 `polygons:` 那一行之后、原有 `FootprintApproach:` 段之前，插入：

```yaml
    # 急停区：紧贴机器人外轮廓。按实测包围盒 x±0.19、y±0.24 外扩 5cm，
    # 留出的余量用于吸收定位误差与麦轮滑移。任何点落入此区立即输出零速度。
    StopZone:  # 急停碰撞区域配置段。
      type: "polygon"  # 区域类型：多边形。
      action_type: "stop"  # 检测到碰撞即完全停止机器人。
      points: "[[-0.24, -0.29], [-0.24, 0.29], [0.24, 0.29], [0.24, -0.29]]"  # 多边形顶点，单位米，机体坐标系。
      visualize: True  # 发布该区域用于可视化，便于在 Foxglove 中核对范围。
      enabled: True  # 是否启用该碰撞区域。
    # 减速区：位于急停区外侧，接近障碍时先限速而非骤停，避免探索途中频繁急刹。
    SlowZone:  # 减速碰撞区域配置段。
      type: "polygon"  # 区域类型：多边形。
      action_type: "slowdown"  # 进入此区域时按比例降低速度。
      points: "[[-0.55, -0.60], [-0.55, 0.60], [0.55, 0.60], [0.55, -0.60]]"  # 多边形顶点，单位米，机体坐标系。
      slowdown_ratio: 0.3  # 进入减速区后速度降为原来的 30%（注意参数名无下划线分隔 slow/down）。
      visualize: True  # 发布该区域用于可视化。
      enabled: True  # 是否启用该碰撞区域。
```

- [x] **Step 5: 放宽 stop_pub_timeout**

```yaml
    stop_pub_timeout: 2.0  # 停止指令持续发布的时间，单位秒。
```

改为：

```yaml
    stop_pub_timeout: 5.0  # 停止指令持续发布的时间，单位秒；探索场景下放宽，避免障碍移开后过早放行。
```

- [x] **Step 6: 清除 scan 源上的死参数**

> **执行时新增（2026-09-18，计划外）**：不属于原计划，实测发现问题后补入。

`collision_monitor` 的 `scan:` 段带 `min_height: 0.15` / `max_height: 2.0`，这对参数**不会被读取**。证据：

- `nav2_collision_monitor/scan.hpp` 的 `Scan` 类只有 `data_sub_`、`data_` 两个成员，无高度成员；`source.hpp` 基类与 `range.hpp` 同样没有。只有 `pointcloud.hpp:97` 有 `double min_height_, max_height_;`，即这对参数专属 `PointCloud` 类观测源。
- 实测：以本参数文件 `configure` 后 `ros2 param list` 只见 `scan.type` / `scan.topic` / `scan.enabled` / `scan.source_timeout`，无高度项。

**这段死配置比一般的死配置更危险**：它与 `src/myexplore/config/scan.yaml` 中真正生效的 `min_height` / `max_height` 同名。将来调扫描高度窗口的人若改到这里，会毫无效果，并误以为碰撞侧的高度过滤已经配过 —— 而 Task 2 Step 7 的分支 B（雷达斜装）恰恰是个需要调高度窗口的场景。

删除这两行，替换为说明注释：

```yaml
      min_height: 0.15  # 参与碰撞检测的最低激光高度，单位米。
      max_height: 2.0  # 参与碰撞检测的最高激光高度，单位米。
      enabled: True  # 是否启用该激光观测源。
```

改为：

```yaml
      # 此处刻意不设 min_height / max_height。这对参数只属于 PointCloud 类观测源
      # （见 nav2_collision_monitor/pointcloud.hpp 的 min_height_/max_height_ 成员），
      # scan 源的 Scan 类并无这两个成员。实测：节点 configure 后
      # `ros2 param list` 只见 scan.type / scan.topic / scan.enabled / scan.source_timeout。
      # 留着它们会有害 —— 真正决定扫描高度窗口的是 myexplore/config/scan.yaml 里的
      # 同名参数，在此处改高度不会有任何效果，反而会让人以为配过碰撞侧的高度过滤。
      enabled: True  # 是否启用该激光观测源。
```

同时在 `tools/check_nav2_params.py` 的 `PATH_CHECKS` 末尾再追加两条，把"已移除"变成可断言的事实：

```python
    # --- scan 源上的死参数已清除（Scan 类不读高度，见 pointcloud.hpp 对比）---
    ("scan 源已移除死参数 min_height", CM + ("scan", "min_height"), None),
    ("scan 源已移除死参数 max_height", CM + ("scan", "max_height"), None),
```

- [x] **Step 7: 运行断言脚本，确认全部通过**

```bash
python3 tools/check_nav2_params.py
```

Expected: 全部 `PASS`，末行 `31/31 通过`（25 项 + 4 项 + 2 项）。

若 `collision_monitor.polygons` 或 `StopZone.action_type` 仍为 `FAIL`，说明 Step 4 的 YAML 缩进不对 —— `StopZone` / `SlowZone` 必须与 `FootprintApproach` 同级（4 个空格），其子键为 6 个空格。

参数名对照（已核对 `nav2_collision_monitor` 的 `polygon.hpp` 与已安装的 `libcollision_monitor_core.so`）：多边形顶点用 `points`，减速比例用 `slowdown_ratio`（**不是** `slow_down_ratio`），激活条件用 `min_points`（旧名 `max_points` 已废弃）。

- [ ] **Step 8: 现场验证急停生效**

重启 nav2 使新配置生效：

```bash
colcon build --packages-select myexplore
source install/setup.bash
ros2 launch myexplore navigation.launch.py
```

在 Foxglove 中显示 `/collision_monitor_state` 与 `/cmd_vel`。

测法：用手推着车（或遥控）让车前方 20cm 处贴上障碍物，然后发一个会撞上去的导航目标：

```bash
ros2 topic echo /collision_monitor_state --once
```

Expected: 状态从 `NORMAL` 变为 `STOPPED`，且：

```bash
ros2 topic echo /cmd_vel --once
```

Expected: `linear` 与 `angular` 全为 0（即使 controller 仍在输出非零的 `cmd_vel_smoothed`）。

对照确认拦截确实发生：

```bash
ros2 topic hz /cmd_vel_smoothed
ros2 topic hz /cmd_vel
```

Expected: 两个话题都仍在发布，但 `/cmd_vel` 内容为零。若 `/cmd_vel` 完全停发，说明是上游断了而非 collision_monitor 拦截，需另行排查。

移开障碍物，确认状态回到 `NORMAL` 且车恢复移动。

- [ ] **Step 9: Commit**

```bash
git add src/myexplore/config/nav2_params.yaml tools/check_nav2_params.py
git commit -m "feat(myexplore): collision_monitor 增加急停区与减速区"
```

---

### Task 6: explore_lite 接入 + explore_bringup 总入口

**Files:**
- Create: `src/myexplore/config/explore_params.yaml`
- Create: `src/myexplore/launch/explore.launch.py`
- Create: `src/myexplore/launch/explore_bringup.launch.py`

**Interfaces:**
- Consumes: `/map`（Task 3）、`/global_costmap/costmap` 与 `NavigateToPose` action server（Task 4）
- Produces: 节点 `explore_node`；订阅话题 `/explore/resume`（`std_msgs/msg/Bool`），发布 `/explore/status`（`explore_lite_msgs/msg/ExploreStatus`）与 `/explore/frontiers`（`visualization_msgs/msg/MarkerArray`）。`explore_bringup.launch.py` 成为 Task 7 中 `mystart.sh` 的调用入口。

- [x] **Step 1: 写 explore_params.yaml**

创建 `src/myexplore/config/explore_params.yaml`：

```yaml
# 根键必须是 explore_node，与 explore.launch.py 中 Node(name="explore_node") 对应。
# 不对应的话参数会被静默忽略，节点跑在代码默认值上。
explore_node:
  ros__parameters:
    robot_base_frame: base_link             # 底盘坐标系，与 URDF 一致。
    # 数据源用 nav2 全局 costmap 而非 /map：slam_toolbox 不发布 map_updates 话题
    # （该话题名源自 ROS1 的 map_server），走 /map 的话增量更新路径是断的。
    costmap_topic: /global_costmap/costmap  # 前沿搜索读取的代价地图话题。
    costmap_updates_topic: /global_costmap/costmap_updates  # 代价地图增量更新话题。
    visualize: true                         # 发布 /explore/frontiers 供 Foxglove 可视化。
    planner_frequency: 0.15                 # 约 6.7 秒决策一次；过于频繁会导致目标反复横跳。
    progress_timeout: 30.0                  # 30 秒无进展即换目标，内置的防卡死机制，不宜调小。
    potential_scale: 3.0                    # 距离惩罚权重。
    orientation_scale: 0.0                  # 朝向惩罚默认关闭；麦轮可横移，开关皆可。
    gain_scale: 1.0                         # 前沿尺寸奖励权重。
    transform_tolerance: 0.3                # TF 时间容忍度，单位秒。
    min_frontier_size: 0.5                  # 过滤过小前沿，避免被噪声诱导钻角落；调大更保守。
    return_to_init: false                   # 本期目标是持续探索，探索完不返航。
    use_sim_time: false                     # 真机无 /clock；上游 launch 默认 true，必须显式设 false。
```

- [x] **Step 2: 写 explore.launch.py**

创建 `src/myexplore/launch/explore.launch.py`：

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("myexplore"), "config", "explore_params.yaml"
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
```

- [x] **Step 3: 写 explore_bringup.launch.py**

创建 `src/myexplore/launch/explore_bringup.launch.py`：

```python
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
```

- [x] **Step 4: 构建并做语法检查**

```bash
for f in src/myexplore/launch/explore.launch.py src/myexplore/launch/explore_bringup.launch.py; do
  python3 -m py_compile "$f" || exit 1
done
python3 -c "import yaml; yaml.safe_load(open('src/myexplore/config/explore_params.yaml')); print('yaml 语法 OK')"
colcon build --packages-select myexplore
source install/setup.bash
ros2 launch myexplore explore_bringup.launch.py --show-args
```

Expected: 列出 `explore` 参数，默认 `'true'`。

- [ ] **Step 5: 验证 explore 开关生效**

```bash
ros2 launch myexplore explore_bringup.launch.py explore:=false
```

另开终端：

```bash
ros2 node list | grep explore
```

Expected: 无输出（探索确实没起）。

Ctrl+C 退出，再起完整版：

```bash
ros2 launch myexplore explore_bringup.launch.py explore:=true
```

Expected: 日志出现 `[explore_node]` 前缀的行。

**核对参数是否真的被读到** —— 必须用 `costmap_topic`，不能用 `use_sim_time`：

```bash
ros2 param get /explore_node costmap_topic
```

Expected: `String value is: /global_costmap/costmap`。

> **为什么这两条命令不等价**（执行时实测更正）：`use_sim_time` 由 `rclcpp::Node` 自动声明，默认值就是 `false`。因此 YAML 根键写错、参数完全没被加载时，它**照样**返回 `False` —— 拿它判断根键是否匹配是无效检查。`costmap_topic` 才有判别力：代码默认值是 `"costmap"`（`costmap_client.cpp:58`），只有 YAML 真正加载才会变成 `/global_costmap/costmap`。

**读不到 `return_to_init` / `min_frontier_size` / `planner_frequency` 属正常**，不是配置问题：`Explore` 的成员初始化列表里先构造 `costmap_client_`，其构造函数会阻塞等待代价地图话题；在 nav2 起来之前，构造函数体（`explore.cpp:65-72`）的 `declare_parameter` 尚未执行。上述四个取自 `CostmapClient` 构造函数的参数与自动声明的 `use_sim_time` 则随时可读。

- [ ] **Step 6: 验证探索启动与状态上报**

```bash
ros2 topic echo /explore/status
```

Expected: 持续输出，`status` 字段为字符串 `exploration_started`。（`ExploreStatus.msg` 里 `status` 是 `string`，值为 `exploration_started` / `exploration_in_progress` / `exploration_paused` / `exploration_complete` / `returning_to_origin` / `returned_to_origin` 这六个字符串常量，**没有数字枚举值**。）

```bash
ros2 topic echo /explore/frontiers --field markers --once | head -20
```

Expected: 有若干 Marker，每个是一次候选前沿。

在 Foxglove 中显示 `/explore/frontiers`（MarkerArray）：

Expected: 机器人周围的 frontier 候选点被标出，其中一个被选中作为目标。

**观察车体**：应开始自主移动，朝未探索区域走。若 `explore/status` 一直不出或为 `exploration_complete`，见 Step 8。

- [ ] **Step 7: 验证人工喊停**

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"
```

Expected: 车立即停下；`ros2 topic echo /explore/status` 变为字符串 `exploration_paused`。

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"
```

Expected: `exploration_started`，车恢复自主移动。

- [ ] **Step 8: 处理前沿耗尽**

若长期运行后 `/explore/status` 变为 `exploration_complete` 且车停下不再动，说明当前地图内的前沿已被探索完。检查是否真的探索完了：

```bash
ros2 topic echo /map --field info --once
```

若地图确实已覆盖全部可达区域，这是正常行为。若明显还有大片未探索区域却报完成，说明 `/global_costmap/costmap` 的未知区域没有正确传递：

```bash
ros2 topic echo /global_costmap/costmap --field info --once
```

确认 `width`/`height` 随车移动增长，且 `track_unknown_space: true` 生效（地图中应存在值为 -1 的未知栅格）。

应对：本项目需求是"一直探索，手动喊停"，若 `exploration_complete` 被过早触发，把 `min_frontier_size` 从 `0.5` 调小到 `0.3`，让更小的前沿也算数。

- [ ] **Step 9: 长时间稳定性观察**

让机器人持续自主探索 15 分钟以上，观察：

- 地图是否持续扩大（`ros2 topic echo /map --field info --once` 的 `width`/`height` 单调增长或有增有减但不回退）
- 是否卡在某个死角反复尝试（`explore/status` 反复在 started/paused 间跳，车原地打转）
- 是否出现来回震荡（频繁在两个相邻目标间往返）

若卡死角：调大 `progress_timeout` 到 `45.0`，给慢速脱困更多时间。
若来回震荡：调小 `planner_frequency` 到 `0.1`，降低目标切换频率。

- [ ] **Step 10: 存图（人工触发）**

```bash
ros2 run nav2_map_server map_saver_cli -f ~/nav_map --ros-args -p save_map_timeout:=10.0
```

Expected: 生成 `~/nav_map.pgm` 与 `~/nav_map.yaml`。

- [ ] **Step 11: Commit**

```bash
git add src/myexplore/config/explore_params.yaml src/myexplore/launch/explore.launch.py src/myexplore/launch/explore_bringup.launch.py
git commit -m "feat(myexplore): 接入 explore_lite 自主探索与总入口 launch"
```

---

### Task 7: mystart.sh 扩展为三阶段

**Files:**
- Modify: `mystart.sh`（重整为三阶段）

**Interfaces:**
- Consumes: `myexplore/launch/explore_bringup.launch.py`（Task 6）
- Produces: 一键启动入口，无下游。

**顺序约束**：`slam_toolbox` 与 nav2 需要 `/scan` 与 odin 的 TF 就绪；`explore_lite` 需要 `/map` 与 nav2 的 `NavigateToPose` action server 就绪。因此前两层后台起、延迟后起第三层，且第三层放前台以便 Ctrl+C 退出。

- [ ] **Step 1: 替换 mystart.sh 全文**

```bash
#!/usr/bin/env bash
#
# 一键启动：系统层 -> 控制层 -> 探索层，三层依次拉起。
#   mysystem  : robot_state_publisher + joint_state_publisher + foxglove_bridge  （后台）
#   mycontrol : odin 雷达 + MID360 + 底盘控制节点                                  （后台）
#   myexplore : slam_toolbox + nav2 + explore_lite                                 （前台）
#
# Ctrl+C 退出时会自动把后台的两个 launch 一起收掉，不留孤儿进程。
#
# 用法：
#   ./mystart.sh                    # 默认各层等待 3 秒 / 8 秒
#   WAIT_SEC=8 ./mystart.sh         # 第一层起得慢时放宽
#   WAIT_SEC2=15 ./mystart.sh       # 雷达与 TF 就绪慢时放宽
#   EXPLORE=false ./mystart.sh      # 只起导航不起探索，用于单点导航验证

cd "$(dirname "$(readlink -f "$0")")" || exit 1  # 切到脚本所在目录，保证从任何路径调用都能找到 install/。

# 环境准备：ROS 本体 + 本工作区，缺一个 ros2 launch 都会报找不到包。
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"  # 优先用当前已 source 的发行版，没设就默认 jazzy。
if [ ! -f "$ROS_SETUP" ]; then  # 如果上面拼出来的路径不存在（比如 ROS_DISTRO 指向了没装的版本）。
    ROS_SETUP="/opt/ros/jazzy/setup.bash"  # 那就退回写死的 jazzy 路径再试一次。
fi  # 结束第一次回退判断。
if [ ! -f "$ROS_SETUP" ]; then  # 回退之后仍然找不到，说明这台机器根本没装 ROS。
    echo "[mystart] 找不到 ROS 环境：$ROS_SETUP" >&2  # 把错误原因打到标准错误，方便重定向时也能看到。
    exit 1  # 直接退出，不继续往下跑。
fi  # 结束第二次检查。
# shellcheck disable=SC1090  # 告诉 shellcheck 别警告“无法跟随非字面量路径的 source”。
source "$ROS_SETUP"  # 载入 ROS 本体环境，提供 ros2 命令和系统自带的包。

if [ ! -f install/setup.bash ]; then  # 本工作区的环境文件不存在，通常意味着还没构建过。
    echo "[mystart] 找不到 install/setup.bash，先跑一次 ./mybuild.sh" >&2  # 提示用户先构建。
    exit 1  # 没构建就没法启动，直接退出。
fi  # 结束工作区检查。
# shellcheck disable=SC1091  # 告诉 shellcheck 别警告“source 的文件此刻不存在”（它是运行时才生成的）。
source install/setup.bash  # 载入本工作区环境，让 ros2 能找到 mysystem / mycontrol / myexplore。

WAIT_SEC="${WAIT_SEC:-3}"    # 系统层起来后等几秒再拉控制层，可用环境变量覆盖。
WAIT_SEC2="${WAIT_SEC2:-8}"  # 控制层起来后等几秒再拉探索层；雷达与 TF 就绪需要更久，所以默认值更大。
EXPLORE="${EXPLORE:-true}"   # 是否启动探索，false 时只起导航，用于单点导航验证。

PIDS=()  # 记录所有后台 launch 的 PID，退出时要靠它们收尾。

# 收到 Ctrl+C 或脚本正常结束时，把后台的 launch 一起关掉。
cleanup() {  # 定义清理函数，注册到 trap 上，脚本无论如何退出都会执行。
    echo  # 先打个空行，避免和前面节点的日志挤在一起。
    echo "[mystart] 正在关闭后台 launch ..."  # 告诉用户后台那些 launch 正在被收掉。
    for pid in "${PIDS[@]}"; do  # 逐个发信号，先全部通知再统一等待，避免串行等待拖慢退出。
        kill -INT "$pid" 2>/dev/null  # 发 Ctrl+C 同样的信号，让 ros2 launch 自己优雅退出。
    done  # 结束第一轮循环。
    for pid in "${PIDS[@]}"; do  # 第二轮统一等待，确保都退干净，不留僵尸进程。
        wait "$pid" 2>/dev/null  # 等待该进程结束。
    done  # 结束第二轮循环。
}  # 结束清理函数定义。
trap cleanup EXIT  # 注册：脚本退出时（包括被 Ctrl+C 打断）自动调用上面的清理函数。

echo "[mystart] 1/3 启动 mysystem（模型 + Foxglove）..."  # 打印进度，让用户知道现在在起哪个层。
ros2 launch mysystem mysystem_love_launch.py &  # 后台启动系统层，末尾的 & 是关键，不然后面就执行不下去了。
PIDS+=($!)  # 记下后台进程的 PID。

echo "[mystart] 等待 ${WAIT_SEC} 秒让其就绪 ..."  # 打印进度，说明接下来要等一会儿。
sleep "$WAIT_SEC"  # 等待，让 robot_state_publisher 先把 robot_description 发布出来。

echo "[mystart] 2/3 启动 mycontrol（odin + MID360 + 底盘）..."  # 打印进度，进入第二个 launch。
ros2 launch mycontrol mycontrol_launch.py &  # 后台启动控制层，注意和上一层的区别是不带前台阻塞。
PIDS+=($!)  # 记下后台进程的 PID。

echo "[mystart] 等待 ${WAIT_SEC2} 秒等雷达与 TF 就绪 ..."  # 雷达出点云、odin 出 odom->imu 都需要时间。
sleep "$WAIT_SEC2"  # 等待，避免 slam_toolbox 起来时还没有 /scan 和 TF 可订阅。

echo "[mystart] 3/3 启动 myexplore（SLAM + nav2 + 探索，explore=${EXPLORE}）..."  # 打印进度，进入第三个 launch。
ros2 launch myexplore explore_bringup.launch.py explore:="${EXPLORE}"  # 前台启动探索层，输出直接显示在终端。

# 第三个 launch 退出后，上面注册的 trap 会自动收掉前两个。

- [ ] **Step 2: 语法检查**

```bash
bash -n mystart.sh && echo "bash 语法 OK"
```

Expected: `bash 语法 OK`

- [ ] **Step 3: 验证 explore 开关**

在雷达与底盘都接好的情况下：

```bash
EXPLORE=false WAIT_SEC2=5 ./mystart.sh
```

Expected: 三层依次启动；第三层日志中无 `explore_node`。Ctrl+C 后终端回到提示符，且：

```bash
pgrep -af "ros2 launch" | wc -l
```

Expected: `0`（无孤儿进程残留）。

- [ ] **Step 4: 完整启动验证**

```bash
./mystart.sh
```

Expected: 三层全部启动，机器人开始自主探索。

另开终端验证全链路：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic echo /explore/status --once
ros2 node list | grep -c explore_node
```

Expected: `/scan` 约 10 Hz；`/map` 约 0.2 Hz；status 为 `exploration_started`；`explore_node` 计数为 `1`。

Ctrl+C 后确认全部收干净：

```bash
pgrep -af "ros2 launch" | wc -l
pgrep -af "livox\|host_sdk_sample\|async_slam_toolbox" | wc -l
```

Expected: 两个都是 `0`。

- [ ] **Step 5: Commit**

```bash
git add mystart.sh
git commit -m "feat: mystart.sh 扩展为系统/控制/探索三阶段启动"
```

---

## 完成标准

全部 7 个任务完成后，应满足 Spec §2 的目标：

- [ ] `./mystart.sh` 一条命令拉起全链路，机器人开始自主移动
- [ ] `/map` 持续更新，边界与实际环境吻合，无双层墙
- [ ] `ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"` 能立即停车
- [ ] 长按 Ctrl+C 后无孤儿进程残留
- [ ] `python3 tools/check_nav2_params.py` 退出码为 0
- [ ] 前方 20cm 贴障碍物时 `/cmd_vel` 输出为零（collision_monitor 生效）

## 遗留项（本期不做，供后续接手）

- `src/mynav` 参数迁移后仅剩空壳，待清理。本期不动以免引入无关改动。
- `src/mysystem/urdf/robot.urdf` 中 `joint_livox_frame` 的 45° 俯仰若经 Task 2 Step 7 确认为真实安装角，需改硬件（把 MID360 装平），否则 `/scan` 有效距离受限。
- 3D 点云地图需先解决 odin 与 MID360 的时钟同步（odin 侧可试 `use_host_ros_time: 2`，即 PTP 平滑偏移对齐，见 `src/odin_ros_driver/include/host_sdk_sample.h:197-198`；注意 `config/control_command.yaml:8` 的注释描述的是模式 1 的行为，有误导）。解决后再评估融合方案。
- 探索完成自动存图：`map_saver` 已在链路上，可加自动化触发。
- 探索完成后切换定位模式：`custom_map_mode: 2`（重定位）+ AMCL 的完整产品形态。
