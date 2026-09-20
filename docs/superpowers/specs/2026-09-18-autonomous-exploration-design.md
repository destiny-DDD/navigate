# 自主探索建图设计

日期：2026-09-18
状态：待评审

## 1. 背景

当前仓库已有的能力：

- 底盘可通过 `/cmd_vel` 遥控驱动（已实测跑通），麦轮全向构型
- odin 驱动以 SLAM 模式运行，提供 `odom`→`imu` TF 和 `/odin1/odometry`
- MID360 驱动可发布点云
- 整棵 TF 树连通，根为 `odom`

缺失的是一条从传感器到自主移动的完整链路。目标功能：**机器人自己驱动自己在未知环境中跑动，持续构建 2D 栅格地图，人只需在必要时喊停。**

### 原始设想与被排除的方案

最初设想是"odin 与 MID360 降频到同频 → 拼接两路点云 → PCL 建 3D 图 → 转 2D"。核实后排除，理由记录如下，供日后回溯：

1. **帧率对齐是伪前提**：两路本来就已经都是 10 Hz（odin `dtof_fps: 100` 即 10 fps，MID360 `publish_freq: 10.0`）。且点云融合靠的是按时间戳各自变换到公共坐标系，与帧率是否相同无关。
2. **时钟未同步（决定性障碍）**：odin 用设备内部时钟打时间戳（`use_host_ros_time: 0`，配置注释明确写了 "without PTP offset correction"），MID360 用自身时钟，`MID360_config.json` 中无任何时间同步配置。两者未对齐时，用 TF 变换点云会在运动中使用错误时刻的位姿，导致重影。
3. **host 侧累积极易丢失回环检测**：odin 没有任何地图话题（13 个 publisher 中无 map），`/odin1/cloud_slam` 是**单帧**点云（`host_sdk_sample.cpp:1067` 的 `publishPC2XYZRGBA(stream, 0)`），只是已用设备 SLAM 位姿变换到 odom 系。设备端经回环优化的地图只存于设备内，仅能通过 `save_map` 导出私有 `.bin`。在 host 上逐帧累积等同纯里程计累积，无回环，长时间探索后 2D 图会出现双层墙，不可用于导航。
4. **两雷达互相照射**：odin 与 MID360 同为主动激光源，可能互相收到对方光束形成固定假障碍。

3D 建图不是不做，而是**本期不做**。它依赖第 2 条的时钟同步问题解决，那是独立的一块工作，不应阻塞主线。详见第 9 节。

## 2. 目标与非目标

**目标**

- 启动后机器人自主探索未知环境，持续构建 2D 占据栅格地图
- 支持人工随时暂停/恢复探索（对应"一直探索，手动喊停"）
- 探索过程有独立的防撞兜底机制

**非目标（本期不做）**

- 3D 点云地图
- 探索完成后自动保存地图文件（`map_saver` 会在链路上，人工触发即可）
- 探索完成后自动切换到定位+导航模式
- 多机协同探索
- 自主回充、巡逻等其它自主行为

## 3. 架构

### 3.1 数据流

```
  odin (host_sdk_sample)                  livox_ros_driver2 (MID360)
  ┌────────────────────┐                  ┌────────────────────┐
  │ custom_map_mode: 1 │                  │ publish_freq: 10.0 │
  │ 仅取位姿           │                  │ frame: livox_frame │
  └─────────┬──────────┘                  └─────────┬──────────┘
            │                                       │ /livox/lidar
  /odin1/odometry + TF odom→imu                       │ (PointCloud2)
            │                                       ▼
            │                          ┌─────────────────────────┐
            │                          │ pointcloud_to_laserscan │
            │                          │ 高度切片 → 2D 投影       │
            │                          └────────────┬────────────┘
            │                                       │ /scan (LaserScan)
            │                                       ▼
            │                          ┌─────────────────────────┐
            └─────────────────────────▶│      slam_toolbox       │
                     (里程计先验)       │  异步建图 + 回环检测     │
                                       └────────────┬────────────┘
                                                    │ /map + TF map→odom
                     ┌──────────────────────────────┼──────────────────┐
                     ▼                              ▼                  ▼
           ┌──────────────────┐        ┌──────────────────┐  ┌──────────────┐
           │  nav2 global     │        │ explore_lite     │  │ Foxglove /   │
           │  costmap         │◀───────│ (explore_node)   │  │ RViz         │
           └────────┬─────────┘  goal  └──────────────────┘  └──────────────┘
                    │ NavigateToPose action
                    ▼
           ┌──────────────────┐
           │ nav2 controller  │ → cmd_vel_nav
           │ velocity_smoother│ → cmd_vel_smoothed
           │ collision_monitor│ → cmd_vel
           └────────┬─────────┘
                    │ /cmd_vel
                    ▼
           ┌──────────────────┐
           │ mycontrol        │ → 串口 → 麦轮
           └──────────────────┘
```

### 3.2 TF 树

三个来源各管一段，无任何坐标系有两个父节点：

```
map ──(slam_toolbox)──▶ odom ──(odin)──▶ imu ──(URDF fixed)──▶ up ──┬──▶ livox_frame
                                                                    └──▶ base_link ──▶ wheel_1..4
```

- `map`→`odom`：仅由 slam_toolbox 发布
- `odom`→`imu`：仅由 odin 发布
- 其余：`robot_state_publisher` 依据 `mysystem/urdf/robot.urdf` 发布

**AMCL 必须不启动。** 它的 `tf_broadcast` 默认为 true，会与 slam_toolbox 争夺 `map`→`odom` 的发布权，导致 TF 树冲突。

### 3.3 cmd_vel 链路

复用的 `nav2_bringup/launch/navigation_launch.py` 已把 controller / behavior_server / bt_navigator 的输出重映射为 `cmd_vel_nav`，因此链路为：

```
controller → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed → collision_monitor → cmd_vel → mycontrol
```

`mycontrol` 订阅的 `/cmd_vel` 实际由 collision_monitor 发出。这是**必须复用 nav2_bringup 而非手写各节点 launch** 的原因之一——重映射链路和生命周期管理都已由它处理。

## 4. 依赖与就绪状态

| 组件 | 状态 | 说明 |
|---|---|---|
| `pointcloud_to_laserscan` | 已装 | `ros-jazzy-pointcloud-to-laserscan 2.0.2`，可执行文件 `pointcloud_to_laserscan_node` |
| `slam_toolbox` | 已装 | `/opt/ros/jazzy`，参考配置 `config/mapper_params_online_async.yaml` |
| `nav2_bringup` | 已装 | 含 planner / controller / bt_navigator / behaviors / lifecycle |
| `nav2_mppi_controller` | 已装 | 支持 `motion_model: Omni` |
| `nav2_collision_monitor` | 已装 | |
| `nav2_map_server` | 已装 | 含 `map_saver`，用于存图 |
| `explore_lite` | **已编译验证** | 位于 `src/m-explore-ros2/explore`，包名 `explore_lite`，可执行文件 `explore`，节点名 `explore_node`。已实测在 Jazzy 上可启动并正常打日志 |
| odin / MID360 / mycontrol | 已就绪 | 无需改动 |

### 4.1 已知待办

```
touch src/m-explore-ros2/map_merge/COLCON_IGNORE
```

`multirobot_map_merge` 为多机地图融合，单机场景不需要，且其 README 自述依赖 `slam_gmapping`（ROS1 包，未移植至 ROS2）。它当前会在编译期下载测试地图并产生失败噪音（走本地代理 SSL 断开），耗时约 49 秒，应屏蔽。

## 5. 逐层设计

### 5.1 odin —— 退化为纯里程计

**不改动任何配置。** `custom_map_mode: 1` 继续运行，但仅使用其 `odom`→`imu` TF 与 `/odin1/odometry`。其内部 3D 地图不使用（也无法导出给 nav2）。

可选调优项（不在本期范围）：`custom_map_mode: 0` 为纯里程计模式，不维护不需要的 3D 地图，资源占用更低，但无回环，长距离漂移更大。待整体跑通后再评估。

**注意**：odin 发布的 odom 话题名是 `/odin1/odometry`，**不是 `/odom`**。现有 nav2 配置中多处按 `/odom` 填写，需修正（见 5.4）。

### 5.2 pointcloud_to_laserscan —— 3D 点云降为 2D 扫描

**为什么用 MID360 而非 odin**：MID360 是规律的 360° 扫描，每个角度都有点，压成 LaserScan 天然合理；odin 是 dToF 非重复扫描，点云在空间上稀疏且非均匀，同角度这帧有下帧无，压成 LaserScan 会产生大量空洞，使 slam_toolbox 的扫描匹配失效。这是传感器工作方式差异，非配置可弥补。

关键配置：

| 参数 | 值 | 说明 |
|---|---|---|
| `target_frame` | `base_link` | 先变换到机体坐标系再做高度切片，使高度窗口有明确物理含义 |
| `min_height` / `max_height` | `-0.15` / `0.25` | 相对 `base_link` 的高度窗口，用于选取水平带状点云。起始值供调参用，非最终值 |
| `angle_min` / `angle_max` | `-π` / `π` | 全向 |
| `range_min` / `range_max` | `0.2` / `20.0` | 起始值；`range_max` 与 slam_toolbox 参考配置的 `max_laser_range: 20.0` 对齐 |
| `scan_time` | `0.1` | 对应 10 Hz |
| `use_inf` | `true` | 无回波方向输出 `inf` 而非直接丢弃 |

**上表的高度窗口为起始值，需实测调优**：`robot.urdf` 中 `joint_livox_frame` 的 origin 带 `rpy="0.00000735 0.78539265 3.14159265"`，即相对 `up` 有约 45° 俯仰和 180° 偏航。这个安装角度会让高度切片的行为偏离直觉，因此必须在第 8.2 步按实机效果调整，起始值只用于让链路先跑起来。此处不阻塞开发。

### 5.3 slam_toolbox —— 2D SLAM 与回环检测

以 `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml` 为基准，改动两处：

- `base_frame`: `base_footprint` → **`base_link`**（URDF 中不存在 `base_footprint`）
- `odom_frame`: `odom` 保持不变（与 odin 的 TF 吻合）

其余保持默认，其中与本期相关的关键项：

- `mode: mapping` —— 在线建图
- `do_loop_closing: true` —— 回环检测，本方案相对于"host 侧累积"的核心优势
- `scan_topic: /scan` —— 与 5.2 的输出对应
- `use_scan_matching: true`
- `resolution: 0.05`
- `transform_publish_period: 0.02` —— `map`→`odom` 的发布周期

采用异步模式（`online_async`）而非同步模式：异步不阻塞扫描回调，在算力有限的车上更稳。

### 5.4 nav2 —— 导航栈

复用 `nav2_bringup` 的 **`navigation_launch.py`**，而不是 `bringup_launch.py`。

这是本设计中容易踩错的一处：`bringup_launch.py` 是个包装层，当 `slam:=False` 时它会 include `localization_launch.py`，而**后者包含 amcl 与 map_server**——AMCL 一旦启动就会与 slam_toolbox 争夺 `map`→`odom` 的发布权。而 `navigation_launch.py` 只含导航栈本体（controller / planner / bt_navigator / behaviors / smoother / velocity_smoother / collision_monitor），**不含 amcl 与 map_server**，正是我们需要的。

附带好处：`collision_monitor` 与 `velocity_smoother` 已由该文件拉起，无需在 `mynav` 中单独启动。

现有 `src/mynav/config/nav2_params.yaml` 需作如下改动。这些是麦轮全向构型的必要修正，其中前三条若漏改会导致横移能力完全失效：

| 位置 | 现值 | 应改为 | 原因 |
|---|---|---|---|
| `controller_server.FollowPath.motion_model` | `DiffDrive` | `Omni` | 麦轮全向 |
| `controller_server.min_y_velocity_threshold` | `0.5` | 减小（如 `0.1`） | 现值为差速车设置，会滤掉横向速度 |
| `velocity_smoother.max_velocity` | `[0.5, 0.0, 2.0]` | `[0.5, 0.5, 2.0]` | Y 分量为 0 会把横移清零 |
| `velocity_smoother.min_velocity` | `[-0.5, 0.0, -2.0]` | `[-0.5, -0.5, -2.0]` | 同上 |
| `velocity_smoother.max_accel` | `[2.5, 0.0, 3.2]` | `[2.5, 2.5, 3.2]` | 同上 |
| `velocity_smoother.max_decel` | `[-2.5, 0.0, -3.2]` | `[-2.5, -2.5, -3.2]` | 同上 |
| `velocity_smoother.odom_topic` | `odom` | `/odin1/odometry` | odin 不发 `/odom` |
| `bt_navigator.odom_topic` | `/odom` | `/odin1/odometry` | 同上 |
| `collision_monitor.base_frame_id` | `base_footprint` | `base_link` | URDF 中不存在该 frame，不改会导致节点起不来 |
| `local_costmap` | 有无用的 `static_layer` 段 | 删除或加入 plugins | 该段存在但不在 `plugins` 列表中，是死配置 |

**保持不变的项**（已适配本场景）：

- `planner_server.GridBased.allow_unknown: true` —— 探索必需，允许路径穿越未知区域
- `global_costmap.plugins` 含 `static_layer`，读取 slam_toolbox 的 `/map`
- `global_costmap` / `local_costmap` 的 `scan.topic: /scan`
- `robot_base_frame: base_link`（多处）
- `collision_monitor.cmd_vel_in_topic: cmd_vel_smoothed` / `cmd_vel_out_topic: cmd_vel`

**待核实项**：`robot_radius: 0.22`。URDF 中四轮位于 (±0.11, ±0.1605)，机体半宽已达 0.16，加上轮宽后需确认 0.22 是否足够。偏小会导致碰撞检查失效。

选用 MPPI controller 而非 DWB：麦轮存在滑移，MPPI 对滑移和全向运动更鲁棒，且原生支持 `motion_model: Omni`。代价是参数较多，以现有配置为起点调优。

### 5.5 explore_lite —— 探索决策

**它不建图、不导航、不控制**，只负责读地图、挑目标、通过 nav2 的 `NavigateToPose` action 发目标、等结果、再挑下一个。

参数文件选择 `config/params_costmap.yaml`（数据源为 nav2 全局 costmap）而非 `params.yaml`（数据源为 `/map`）。原因：`params.yaml` 配了 `costmap_updates_topic: map_updates`，而 **slam_toolbox 不发布 `map_updates`**（该话题名源自 ROS1 的 map_server），增量更新路径是断的；nav2 全局 costmap 则会正常发布 `costmap_updates`。

参数调整：

| 参数 | 默认 | 建议 | 说明 |
|---|---|---|---|
| `robot_base_frame` | `base_link` | 保持 | 与 URDF 一致 |
| `use_sim_time` | **`true`**（launch 默认） | 必须显式设 `false` | 真机上不存在 `/clock`，保持 true 会导致节点静默空转 |
| `return_to_init` | 代码默认 `false`（`params.yaml` 中被人为改成 `true`） | `false` | 本期目标是持续探索，不需探索完自动返航。注意 `params_costmap.yaml` 未设此项，即走代码默认的 `false`，正合需求；若误用 `params.yaml` 则会变成自动返航 |
| `planner_frequency` | 0.15 | 保持 | 约 6.7 秒决策一次；过于频繁会导致目标反复横跳 |
| `progress_timeout` | 30.0 | 保持 | 30 秒无进展即换目标，是内置的防卡死机制，不宜调小 |
| `min_frontier_size` | 0.5（`params_costmap.yaml`） | 保持 | 过滤过小前沿，避免被噪声诱导钻角落。调大更保守 |
| `potential_scale` | 3.0 | 保持 | 距离惩罚 |
| `gain_scale` | 1.0 | 保持 | 前沿尺寸奖励 |
| `orientation_scale` | 0.0 | 保持 | 朝向惩罚默认关闭；麦轮可横移，开关皆可 |

**启动方式**：不使用其自带的 `explore.launch.py`——该文件把参数文件写死为 `params.yaml`，且 `use_sim_time` 默认 `true`。改为在 `mynav` 的 launch 中直接以 `Node` 起 `explore_lite` 的 `explore` 可执行文件。

**人工喊停**（本期需求的核心交互）：

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"   # 暂停
ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: true}"    # 恢复
```

**状态可观测性**：`explore_lite` 发布 `explore/status`（`explore_lite_msgs/ExploreStatus`），状态值包括 `exploration_started` / `exploration_paused` / `exploration_complete` / `returning_to_origin` / `returned_to_origin`。应订阅此话题判断节点是否卡死，比读日志可靠。`explore/frontiers`（MarkerArray）可用于可视化候选前沿。

### 5.6 collision_monitor —— 安全兜底

需求是"一直探索"，因此这层不是可选项。它直接拦在 `cmd_vel_smoothed` 与最终 `cmd_vel` 之间，是独立于探索逻辑的最后一关。

在现有配置基础上扩充 `polygons`，至少包含：

- **急停区**：`action_type: stop`，紧贴机器人轮廓，检测到障碍立即停
- **减速区**：`action_type: slowdown` / `limit`，在急停区外侧，接近障碍时限速
- 保留现有的 `FootprintApproach`（`action_type: approach`）

`observation_sources` 使用同一份 `/scan`。

## 6. 新包 mynav

| 项 | 值 |
|---|---|
| 包名 | `mynav` |
| 构建类型 | `ament_cmake`（仅安装 launch/config，无编译目标） |
| 依赖 | `slam_toolbox`, `nav2_bringup`, `nav2_collision_monitor`, `pointcloud_to_laserscan`, `explore_lite`, `livox_ros_driver2`, `odin_ros_driver` |

目录结构：

```
mynav/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── explore_bringup.launch.py    # 总入口：依次拉起下列各项
│   ├── slam.launch.py               # slam_toolbox
│   ├── scan.launch.py               # pointcloud_to_laserscan
│   └── navigation.launch.py         # include nav2 navigation_launch.py + explore_lite
├── config/
│   ├── slam_toolbox.yaml
│   ├── nav2_params.yaml             # 自 mynav 迁移并修正
│   ├── explore_params.yaml
│   └── collision_monitor.yaml
└── rviz/
    └── explore.rviz
```

**关于 `mynav`**：其 `config/nav2_params.yaml` 迁移至 `mynav` 后，`mynav` 将只剩空壳。本期不动它（避免引入无关改动），但需记录它为待清理项。

## 7. 启动顺序

`mystart.sh` 扩展为三阶段：

| 阶段 | 内容 | 方式 |
|---|---|---|
| 1 | `mysystem`：`robot_state_publisher` + `joint_state_publisher` + `foxglove_bridge` | 后台，现状不变 |
| 2 | `mycontrol`：odin + MID360 + 底盘控制 | 后台（现为前台，需调整） |
| 3 | `mynav`：slam_toolbox + nav2 + explore_lite | 前台 |

顺序约束：slam_toolbox 与 nav2 需要 `/scan` 与 odin 的 TF 就绪；explore_lite 需要 `map` 与 nav2 的 `NavigateToPose` action server 就绪。采用后台启动前两阶段、延迟后启动第三阶段，延迟值（现为 `WAIT_SEC=3`）需按实机调整。

## 8. 验证方法

按层验证，逐层确认后再上下一层。**这是本设计的核心工程原则——四层串行依赖，一起铺会导致无法定位故障层。**

1. **TF 树完整性**
   `ros2 run tf2_tools view_frames`，确认 `map`→`odom`→`imu`→`up`→`base_link` 与 `livox_frame` 链路完整，且每个 frame 只有一个父节点。

2. **`/scan` 质量**
   在 Foxglove / RViz 中显示 `/scan`，缓慢原地旋转，确认：全向覆盖、无大面积空洞、地面与天花板未被误标为障碍、距离读数与实际相符。**此步不通过则后续全部无意义**——slam_toolbox 的建图质量完全取决于它。

3. **slam_toolbox 建图**
   手动遥控机器人走一小圈回到起点，观察 `/map`：边界应与实际吻合，回到起点时墙体不应出现双层。双层即回环未生效。

4. **nav2 单点导航**
   在 RViz 中用 "2D Pose Estimate" 设定初始位姿（此步仅影响显示，slam_toolbox 的 `map` 原点即起点），用 "2D Goal Pose" 手动发一个目标，确认机器人能规划并到达。**在接入探索之前必须单独验证通过**，否则无法区分故障来自导航还是探索。

5. **explore_lite 接入**
   起 `explore_node`，确认 `explore/status` 变为 `exploration_started`，`explore/frontiers` 有候选点，机器人开始自主移动。发布 `explore/resume` 为 `false` 确认能停。

6. **长时间稳定性**
   让机器人持续探索较长时间，观察：地图是否持续扩大、是否卡在死角、是否出现来回震荡。

## 9. 已知风险

| 风险 | 影响 | 应对 |
|---|---|---|
| `livox_frame` 的 45° 俯仰安装 | `/scan` 高度切片参数需重新确定，可能覆盖不到有效水平面 | 第 8.2 步实机确认 |
| `robot_radius: 0.22` 可能偏小 | 碰撞检查失效 | 依 URDF 尺寸核算 |
| odin 的 SLAM 位姿作为 slam_toolbox 的里程计输入 | 双重 SLAM，行为需观察 | 若出现异常，可试 `custom_map_mode: 0` 让 odin 退为纯里程计 |
| `explore_lite` 目标包为 "Humble and newer"，非 Jazzy 原生 | 已实测可编译可启动；但深层功能（回环、边界情况）未验证 | 第 8.5 步暴露 |
| 麦轮滑移导致里程计与实走不符 | MPPI 可容忍；严重时影响建图 | 观察 `/odin1/odometry` 与实际位移的偏差 |
| `explore_lite` 前沿耗尽时的行为 | 需求为"一直探索"，耗尽时是否静默等待未知 | 第 8.6 步观察；必要时订阅 `explore/status` 的 `exploration_complete` 做外部处理 |

## 10. 后续可做（本期不做）

- **3D 点云地图**：需先解决 odin 与 MID360 的时钟同步。odin 侧可改用 `use_host_ros_time: 2`（代码中该模式为 PTP 平滑偏移对齐，见 `host_sdk_sample.h:197-198`；注意配置文件中该行注释描述的是模式 1 的行为，略有误导），MID360 侧需对应机制。解决后再评估融合与建图方案。
- **探索完成自动存图**：`map_saver` 已在链路上，可加自动化触发。
- **探索完成后切换定位模式**：`custom_map_mode: 2`（重定位）+ AMCL 的完整产品形态。
- **清理 `mynav`**：参数迁移完成后其仅剩空壳。
