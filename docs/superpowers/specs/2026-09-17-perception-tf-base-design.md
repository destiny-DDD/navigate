# 感知层与 TF 底座 — 设计

日期：2026-09-17
状态：待评审

## 1. 背景

`nav` 是一个 ROS 2 差分驱动导航小车的 workspace。当前：

- `src/mysystem` 只有描述层：URDF、mesh、rviz 配置，以及两个 launch（RSP + `joint_state_publisher` + rviz/foxglove）。
- `src/mynav` 只有一个 `config/nav2_params.yaml`，没有任何源码，`CMakeLists.txt` 里也没有任何 `install()` 规则。
- `src/mycontrol` 只有两个 libxr 的 MCU 固件模块头文件，不是 ROS 节点。
- 两个传感器驱动是外部拉取的：`src/livox_ros_driver2`（MID360）与 `src/odin_ros_driver`（Odin1）。

结论：仓库里**不存在**任何里程计、TF 或传感器融合的代码流。nav2 配置引用的话题与坐标系（`/odom`、`/scan`、`base_footprint`）没有任何生产者。本设计要建立的就是这个缺失的感知层与 TF 底座。

## 2. 目标

让 TF 树自洽、`odom → base_link` 可达、MID360 点云能进入 nav2 代价地图，并可在 Foxglove 中观察。这是后续所有导航功能的地基。

## 3. 范围

### 在范围内

1. URDF 根坐标系调整，使 TF 树能以 Odin 的 `odom → imu` 为接入点。
2. Odin1 与 MID360 驱动的配置调整（帧、时间基准、点云格式），以**不修改 vendored 文件**的方式施加（§5.0）。
3. 时间基准统一（两个传感器与 TF 在同一时间轴上）。
4. `/odom` 话题的补齐（纯配置）。
5. nav2 代价地图观测源配置，以及 `nav2_params.yaml` 中不一致坐标系的清理。
6. 一把 bringup launch 把驱动、RSP、foxglove 串起来。
7. 修复 `mynav` 缺失 `install()` 规则的问题。

### 明确不在范围内

- **nav2 参数调优**（代价地图分辨率、膨胀半径、规划器/控制器参数等）。只保证观测源接得上且可验证。
- **AMCL 与全局定位。** 定位由 Odin1 主导，本阶段不开 AMCL。
- **`map` 坐标系。** 见 §5.2。
- **转盘（`joint_up`）角度上报。** 见 §4.3。
- 轮速里程计（硬件不提供）。
- RGB 图像链路。

## 4. 关键约束与前提

### 4.1 唯一的里程计源是 Odin1

底盘不提供轮速计。所有位姿信息来自 Odin1 的视觉惯性里程计。因此不存在"多源融合"这一步 —— 本设计做的是**坐标变换与接口适配**，不是滤波器调参。

### 4.2 Odin1 主导定位

`custom_map_mode: 1`（SLAM 建图模式）。地图由 Odin 维护。本阶段不引入 AMCL。

### 4.3 转盘关节本阶段不动

`joint_up` 在硬件上是一个连续旋转关节，URDF 中类型正确。本阶段转盘保持停转（角度 0），**不做 MCU 角度上报的桥接**。这是本设计的显式前提。

后果：若托架被转动，`base_link` 的朝向会偏掉同样的转角。这是已知的、被接受的限制，由后续独立工作消除。

因该前提成立，`joint_up` **保持 `continuous` 不改**。理由：

- 保持原样意味着将来做转盘时 URDF 无需改动（向前兼容）。
- 托架停在 0 位时，`joint_state_publisher` 发布的 0 就是正确值，行为与 `fixed` 完全等价。
- 代价：`imu → up` 这条处于 TF 链**正中间**的边走 `/tf`（10 Hz）而非 `/tf_static`，其存在依赖 `/joint_states` 持续发布。此项列入验收清单。

> 留给后续工作的提示：反转关节时旋转方向会随之反向（新角 = −旧角）。将来接 MCU 角度时必须确认符号，否则转盘一动 `base_link` 会**朝反方向**偏。

### 4.4 `base_link` 已在地面高度

由仓库现有数据核实，无需 `base_footprint`：

- 轮子网格 `meshes/wheel_1.stl` 的 x/y 包围盒为 ±0.07617 m，即**轮半径 = 0.07617 m**。
- 四个轮关节的 origin 为 `±0.11 ±0.1605 0.0762`，即**轮心在 base_link 上方 0.0762 m**。

轮半径与轮心高度相等，接地点正好在 `base_link` 的 z = 0。`base_footprint` 将是恒等变换，属多余坐标系。

现有 `nav2_params.yaml` 中 `base_footprint`（AMCL、collision_monitor）与 `base_link`（costmap）混用，本身是配置缺陷，本次一并统一为 `base_link`。

## 5. 设计

### 5.0 配置归属原则：不改动 vendored 仓库中的任何文件

`src/livox_ros_driver2` 与 `src/odin_ros_driver` 都是从上游拉取的外部仓库，此前已有过重新拉取的操作。直接修改它们内部的文件，会在下次拉取时被静默覆盖。

因此本设计的所有改动都通过**参数覆盖**或**自有配置文件**完成：

- MID360：`xfer_format` 与 `frame_id` 本就是节点的 ROS 参数，由我们的 bringup 直接传入，不编辑上游 launch。
- Odin1：**参数覆盖行不通** —— 该节点不读任何 ROS 参数，配置路径在编译期就焊死在 vendored 目录上。改用**启动时覆写**，见 §5.4。这是本原则的唯一例外。
- SDF/URDF：`src/mysystem` 是本仓库自己的包，可直接修改。

两个驱动都是主仓库直接跟踪的普通文件（不是 submodule，也没有被 gitignore），因此任何改动都会出现在 `git status` 里 —— 是可见的，不是静默的。

### 5.1 坐标系约定

| 坐标系 | 含义 | 发布者 |
|---|---|---|
| `odom` | Odin 的世界坐标系，本阶段为 TF 树根 | Odin 驱动 |
| `imu` | Odin1 的 IMU（也是本阶段 URDF 的根） | Odin 驱动（`odom → imu`） |
| `up` | 传感器托架 | `robot_state_publisher` |
| `base_link` | 底盘，接地点在其 z = 0 | `robot_state_publisher` |
| `livox_frame` | MID360 | `robot_state_publisher` |
| `lidar` | Odin1 的 dToF 激光 | Odin 驱动（`imu → lidar`） |
| `camera_0` | Odin1 的相机 | Odin 驱动（`odom → camera_0`） |
| `wheel_1..4` | 四个轮子（叶子节点） | `robot_state_publisher` |

机器人本体坐标系统一为 **`base_link`**，弃用 `base_footprint`（依据 §4.4）。

### 5.2 TF 树

目标形态：

```
odom                                    ← Odin 的世界系（树根）
├─ imu                                  ← Odin 的 odom→imu
│   ├─ lidar                            ← Odin 的 imu→lidar
│   └─ up                               ← 以下由 robot_state_publisher 从 URDF 发布
│       ├─ base_link
│       │   └─ wheel_1..4
│       └─ livox_frame                  ← MID360
└─ camera_0                             ← Odin 的 odom→camera_0
```

要点：

- **树根是 `odom`，没有 `map`。** Odin 仅在重定位模式下广播 `odom → map`（且方向是反的：`map` 作子节点）。本阶段跑 SLAM 模式，不发 `odom → map`，因此树中没有 `map` 坐标系。将来启用重定位模式时才需要处理 `map`，届时必须解决方向反转问题。
- **`base_link` 位于 `imu` 之下**，这是本方案能成立的关键：Odin 直接提供 `odom → imu`，而 URDF 提供 `imu → up → base_link`，两者拼起来就是完整的链。
- **每条边恰好一个父节点**，无环无多父。这是验收的硬性断言。

### 5.3 URDF 改动

**这是本次唯一的实体结构改动。** 把 URDF 的根从 `base_link` 换成 `imu`：

| 关节 | 现状 | 改为 |
|---|---|---|
| `joint_imu` | parent=`up`，child=`imu`，type=fixed，xyz `0.0768 0.025 0.063`，rpy `0 0 0` | **删除**，由下一条取代 |
| 新增 `joint_imu_up` | — | parent=`imu`，child=`up`，type=fixed，xyz `-0.0768 -0.025 -0.063`，rpy `0 0 0` |
| `joint_up` | parent=`base_link`，child=`up`，type=**continuous**，xyz `0 0 0.1039`，axis `0 0 1` | parent=`up`，child=`base_link`，type=**continuous**，xyz `0 0 -0.1039`，axis `0 0 1` |
| `joint_livox_frame` | parent=`up`，child=`livox_frame`，fixed，xyz `-0.0808 0 0.0951`，rpy `0.00000735 0.78539265 3.14159265` | **不变** |
| `joint_wheel_1..4` | parent=`base_link`，continuous | **不变** |

数值自洽性（这些 origin 的 rpy 全为 0，逆变换即取负）：

- 原：`imu` 在 `base_link` 下 = `(0,0,0.1039) + (0.0768, 0.025, 0.063)` = `(0.0768, 0.025, 0.1669)`
- 新：`base_link` 在 `imu` 下 = `(-0.0768, -0.025, -0.063) + (0,0,-0.1039)` = `(-0.0768, -0.025, -0.1669)` ✓ 互为相反数

link 自身的 `visual`/`collision`/`inertial` 的 origin 都表达在各自的 link 坐标系内，反转树结构不影响它们，无需改动。

### 5.4 Odin1 配置改动

**做法：** 将上游 `src/odin_ros_driver/config/control_command.yaml` 复制一份到 `mynav/config/odin_control_command.yaml` 由我们拥有；bringup 在拉起节点**之前**把它覆写回 `src/odin_ros_driver/config/control_command.yaml`。

**为什么"用参数指向我们的副本"行不通：** `host_sdk_sample` 不读任何 ROS 参数。它的配置路径在编译期由 `__FILE__` 推出（`get_package_source_directory()` 从 `src/host_sdk_sample.cpp` 向上找 `package.xml`），即 `<源码树>/config/control_command.yaml`，运行时无法改变。上游 launch 里传的 `config_file` 参数是**装饰性的**，节点从不读取它 —— 上游自己的 workflow 也没有做到它 launch 所暗示的事。依据见 §9。

覆写在 `generate_launch_description()` 内完成，早于任何节点启动，因此不存在竞态。上游文件本身不改动，只是每次启动被我们的副本覆盖。

这仍然守住了 §5.0 要防的东西：**真相源是我们的副本**，vendored 文件只是一个每次启动重新施加的输出。重新拉取覆盖了它也不会静默生效 —— 下一次启动会再覆写回去。代价是每次启动都会改写 vendored 目录中的这一个文件（`git status` 会显示）；该目录本来就会被驱动自身写入（设备连接时会在此创建 `Conn_*/` 日志目录、备份 `calib.yaml`，`host_sdk_sample.cpp:1383-1400`），所以写入这里与驱动的既有行为一致。

该 yaml 被 `host_sdk_sample` 整体读入作为参数表（C++ 侧逐键 `get_key_value`），因此**我们的副本必须与上游保持键集合完整**：缺少的键会静默退回代码内默认值。对策见 §6 第 5 条。

| 参数 | 现值 | 改为 | 原因 |
|---|---|---|---|
| `send_odom_baselink_tf` | `1` | **保持 `1`** | 本方案正是接入它的 `odom → imu` |
| `use_host_ros_time` | `0` | **`1`** | 统一时间基准，见 §5.9 |
| `custom_map_mode` | `1` | **保持 `1`** | SLAM 建图模式 |
| `sendrgb` / `sendrgbcompressed` | `1` | **保持** | 1600×1296 的 RGB 占带宽，但不排除 Odin 的视觉部分依赖它，不做无依据的改动；列为观察项 |
| `tf_extra_publish_rate` | `0` | 先保持 | 若实测发现 TF 更新率不足导致外插报错，再用它提高 TF 发布率 |

### 5.5 MID360 配置改动

**做法：** 由 bringup 直接启动 `livox_ros_driver2_node` 并传入参数，不使用也不修改上游的 `launch_ROS2/msg_MID360_launch.py`（依据 §5.0）。

| 参数 | 现值（上游 launch） | 改为 | 原因 |
|---|---|---|---|
| `xfer_format` | `1`（`CustomMsg`） | **`0`（`PointCloud2`）** | nav2 代价地图吃不了 `CustomMsg`。改这个可以直接省掉一个转换节点 |
| `frame_id` | `'livox_frame'` | **不变** | 已与 URDF 中的 `livox_frame` 一致 |
| `multi_topic` | `0` | 不变 | 单雷达，话题为 `/livox/lidar` |
| `publish_freq` | `10.0` | 不变 | |
| `user_config_path` | 相对路径 `../config/MID360_config.json` | 改为指向**已安装**的绝对路径 `get_package_share_directory('livox_ros_driver2')/config/MID360_config.json` | 上游用的是相对 launch 文件的路径，我们自己的 launch 不在同一目录下，必须改用绝对路径 |
| 时间戳 | — | **无需改动** | 默认已是主机时间，见 §5.9 |

`MID360_config.json` 本身不改动（其中已无 timesync 段，即 NoSync，正合所需）。

### 5.6 `/odom` 接口（纯配置补齐）

本方案不产出 `/odom`，但 nav2 的 `bt_navigator` 与 `collision_monitor` 都订阅它（见 `nav2_params.yaml` 的 `odom_topic`）。用 launch 里一条重映射补上：

```python
remappings=[('/odin1/odometry', '/odom')]
```

**必须记录在案的近似代价：**

- `child_frame_id` 是 `imu` 而非 `base_link`。
- `twist` 的线性速度表达在 `imu` 坐标系中，与 `base_link` 相差一个约 0.1 m 的力臂；转速较大时速度误差可达 ~0.1 m/s 量级。

对速度反馈类的消费够用；对精度敏感的用途不够。将来若需要精确 `/odom` 或引入轮速计，升级路径是引入一个专门的适配节点（届时见 §8 风险 5 的限制）。

### 5.7 代价地图观测源与 nav2 配置清理

文件：`src/mynav/config/nav2_params.yaml`，做**最小正确改动**：

1. 代价地图 `observation_sources`：从 `scan` 换成 `/livox/lidar`，并声明 `data_type: PointCloud2`，补全 `marking` / `clearing`、`min_obstacle_height` / `max_obstacle_height`、`obstacle_max_range` / `raytrace_max_range`。
   - 这些键在 `nav2_params.yaml` 中已存在（原属 `scan` 源），改动是**改指向与取值**而非新增结构。
   - 取值起点为 nav2 官方默认值（`obstacle_max_range: 2.5`、`raytrace_max_range: 3.0`、`min_obstacle_height: 0.0`、`max_obstacle_height: 2.0`）。**调优不在范围内**，此处只保证语义正确、能被代价地图消费。
2. 全局代价地图的 `global_frame`：本阶段用 `odom`（树中无 `map`）。
3. 坐标系清理：所有 `base_frame_id` / `robot_base_frame` / `base_frame` 统一为 `base_link`，删除 `base_footprint`。
4. 死引用清理：AMCL 块的 `scan_topic`、`collision_monitor` 的 `scan` 观测源、`scan_frame_id`。
5. AMCL 块本身保留但不启动（本阶段不用，将来重定位时可能复用）。

**不做参数调优**，只保证观测源接得上且可验证。

### 5.8 Launch 组合

在 `mynav` 下新增一把 bringup launch，拉起：

- MID360 驱动（`livox_ros_driver2`），按 §5.5 直接传参
- Odin1 驱动（`odin_ros_driver`），按 §5.4 指向我们的配置副本，并施加 §5.6 的重映射
- `robot_state_publisher` + `joint_state_publisher` + `foxglove_bridge`（复用 `mysystem` 现有 launch）
- 上述 `/odin1/odometry → /odom` 重映射

**关键点一：直接启动 `host_sdk_sample`，不使用上游的 `odin1_ros2.launch.py`。** 两个原因：

1. 重映射必须只施加在驱动节点上。上游 launch 里的 `cloud_reprojection_ros2_node` 会订阅 `/odin1/odometry`，若驱动端的发布被重映射到 `/odom`，该节点将收不到数据。不启动它即可回避。
2. 上游 launch 还会一并启动三个辅助节点（`pcd2depth_ros2_node`、`cloud_reprojection_ros2_node`、`image_overlay_node`）与 `rviz2`。三个辅助节点的输出流在配置里本就是关闭的（`senddepth: 0`、`sendreprojection: 0`、`sendoverlay: 0`），本阶段不需要。

注意 `host_sdk_sample` **不接受任何 ROS 参数** —— 既不读 `config_file`，也不读 `calib_file_path`（后者只被注入给三个辅助节点）。配置路径与运行目录都由节点在 C++ 侧自行解析。因此我们的 launch **不注入任何参数**，改为在 `generate_launch_description()` 中先执行 §5.4 的配置覆写，再拉起节点。

覆写的目标路径必须是**驱动实际会读的那个路径**（源码树的 `src/odin_ros_driver/config/`），不是 `install/` 下的 share 目录 —— 后者驱动从不读。目标路径做成一个 launch 参数（默认由 `COLCON_PREFIX_PATH` 推出工作区根再拼 `/src/odin_ros_driver`），便于覆盖与排查；若目标目录不存在，launch 必须**明确报错**而不是静默跳过。

**关键点二：必须修复 `src/mynav/CMakeLists.txt` 缺失 `install()` 的问题。** 目前该文件只有 `ament_package()`，`config/`（以及即将新增的 `launch/`）根本不会被安装到 `share/`，所有配置都无法加载。

放在 `mynav` 而非新建包：它本就是本仓库中承载 nav 相关配置的地方，且已有 `config/`。

### 5.9 时间基准（本设计中最容易静默失败的一环）

如果传感器时间戳与 TF 时间戳不同源，代价地图用点云时间戳去查 TF 会**外插失败**，点云被静默丢弃 —— 表象是"点云进不了代价地图"，极易被误判成 nav2 参数问题。

已核实的两个事实：

- **MID360 默认（无 PTP）的时间戳就是宿主机的系统时间。** `src/livox_ros_driver2/src/comm/pub_handler.cpp:265-274` 中，当 `timestamp_type` 既非 gPTP/PTP 也非 GPS 时，直接返回 `std::chrono::high_resolution_clock::now()`（主机收包时刻）；该值经 `base_time` 传到 `src/livox_ros_driver2/src/lddc.cpp:310` 成为点云的 `header.stamp`。而 `config/MID360_config.json` 中**没有任何 timesync 配置段**，即处于 NoSync 状态。
- **Odin1 当前用的是设备时间戳**（`use_host_ros_time: 0`）。

两者不同源，必须统一。改 `use_host_ros_time: 1` 让 Odin 改用主机时间。

附带说明：MID360 的时间戳是**主机收包时刻**而非采集时刻，含网络/USB 抖动。对 10 Hz 的代价地图观测可接受，记录在案。

## 6. 测试策略

零代码方案不等于不可测。可自动化的部分：

1. **逆变换数值校验（核心回归项）。** 反转 URDF 根后，用 tf2 查询 `imu → base_link`，断言其结果等于原 URDF 中 `base_link → imu` 的逆。这是本设计唯一具有真正回归价值的不变量 —— 一旦有人手改 URDF 里的外参，这个测试会立刻发现。
2. **TF 树结构断言。** `check_urdf` 通过；用 `tf2_tools view_frames` 或直接查询 tf2 buffer，断言：树中无多父节点、无环、预期坐标系集合与 §5.1 一致。
3. **`/odom` 与 TF 的一致性。** 断言 `/odom` 的位姿与同一时刻 `odom → base_link` 的 TF 在容差内一致（这项会暴露 §5.6 的力臂近似）。
4. **时间戳一致性。** 录一段 bag，断言 `/livox/lidar` 与 `/tf` 的时间戳同源（数量级与系统性偏移均在同一基准内）。
5. **配置副本完整性检查。** 断言 `mynav/config/odin_control_command.yaml`：

   (a) `use_host_ros_time == 1` —— §5.9 的全部意义所在；
   (b) 键集合 ⊇ 一份**冻结的键清单**（当前上游的 44 个键，写死在测试里）。

   理由：该 yaml 被整体读入作为参数表，**缺键会静默退回代码内默认值** —— 这是最难察觉的一类失效。这里刻意**不**采用"与上游文件比对"的写法：§5.4 的覆写机制会让 vendored 文件变成我们自己的副本，比对对象会自我指涉；冻结清单没有这个问题，且足以抓住"编辑时误删一个键"。

   顺带记录（不是待办）：驱动自身的白名单（`yaml_parser.h` 的 `allowed_key_w_int_val` / `_str_val` / `_float_val`，共 42 个键）中含 `log_devel` 与 `showfps`，而**上游 yaml 本来就没有这两个键** —— 它们当前就在静默使用代码默认值。我们的副本沿用上游，不擅自补。

## 7. 验收标准

1. `tf2_tools view_frames` 输出的树形与 §5.2 一致：`odom` 为根，无多父、无环。
2. `odom → base_link` 可用，且机器人移动时在 Foxglove 中正确跟随（**前提：托架停在 0 位**，见 §4.3）。
3. MID360 点云以 `PointCloud2` 发布，且能在 Foxglove 中与 TF 树正确贴合。
4. 点云能进入 nav2 代价地图，在 Foxglove 中可见障碍（不要求调优后的质量）。
5. `/odom` 有数据。
6. **链路方向正确性验证（不可省略）：** 临时用手动方式往 `/joint_states` 发一个已知的 `joint_up` 角度，断言 Foxglove 中 `base_link` 的朝向**确实随之偏转**。此项验的是"`imu → up → base_link` 这条链是通的且方向正确"，而不是角度来源 —— 它是后续做转盘上报时唯一的地基。
7. `ros2 topic hz /joint_states` 持续有输出（保证 §4.3 提到的中间动态边的存活）。

## 8. 风险与待实测确认项

按优先级排列。第 3、4 条原本标为"待实测"，现已靠读源码静态解决，**不需要探针**。

1. **已确认** ~~URDF 的 `imu` link 是否就是 Odin 的 IMU~~ —— 已由用户确认是。这是整棵树成立的前提，现已消除。
2. **已确认** ~~`joint_up` 是否该改为 `fixed`~~ —— 硬件上确实是转动关节，本阶段不动，保持 `continuous`（§4.3）。
3. **已确认（读代码）** ~~`use_host_ros_time: 1` 是否真的让 Odin 的**全部**输出（尤其 TF）都改用主机时间~~ —— **是**。三条 TF 的 `header.stamp` 全部取自 `msg.header.stamp`（`host_sdk_sample.h:1377`、`1483-1485`），而该 stamp 来自 `make_aligned_stamp()`（`host_sdk_sample.h:1206`）—— 正是被 `use_host_ros_time` 控制的那个函数，`== 1` 时直接返回 `node->now()`。结论：§5.9 的取值 `1` 成立，**不需要**退到 `2`。
4. **已确认（读代码）** ~~`send_odom_baselink_tf` 是否会**连带**门控 `imu → lidar` 与 `odom → camera_0`~~ —— **会**。这两条 TF 的发布语句物理上嵌在 `if (getRosNodeControl()->sendOdomBaseLinkTF())` 块内部（`host_sdk_sample.h:1372` 开块，`tf_til` / `tf_wc` 在 `1422-1486`）。我们保持该开关为 `1`，因此 §5.2 中三条 Odin 边都会存在，**设计不变**。
   - 补充条件：这两条边还额外要求 `has_cached_extrinsics_` 为真，即设备连上并回传标定之后才出现 —— 是启动暂态，不是缺失。排查"树里没有 lidar"时先看这个。
5. **B 方案是锁定的，不可叠加。** 将来引入适配节点（补精确 `/odom`，或接轮速计，或做转盘上报）时，**不能直接在本设计的 TF 树上叠加一个 `odom → base_link`** —— 那会让 `base_link` 出现两条路径（`odom → base_link` 与 `odom → imu → up → base_link`）从而构成 TF 环。届时必须同时把 URDF 的根改回 `base_link`，或关闭 Odin 的 TF 广播。升级不是加法，是替换。
6. **转盘在导航中旋转时 Odin 的里程计质量。** Odin1 的 IMU 位于托架上，其里程计位姿是**托架**在 `odom` 下的位姿；链上减去托架转角才还原底盘，数学上自洽，但托架快速旋转会使 Odin 的 SLAM 质量下降，这不是 TF 层能修的。留待后续转盘工作时实测。
7. **Odin 在重定位模式下的 `map → odom` 方向反转。** 本阶段无 `map`，不受影响；启用重定位时这是必须解决的第一个问题。
8. **§5.4 的覆写依赖源码树存在。** 覆写目标默认由 `COLCON_PREFIX_PATH` 推出「工作区根 `/src/odin_ros_driver/config/`」。若从**只有 `install/`、没有源码树**的机器启动，该目录不存在。对策：launch 必须明确报错而不是静默跳过；目标路径是 launch 参数，可显式覆盖。本阶段是单机开发，不构成实际阻碍。

   同类但更隐蔽的一条：`get_package_source_directory()` 用的是**编译期的** `__FILE__`，所以驱动被编译时工作区在哪里，它就永远读哪里。工作区一旦被移动，驱动仍会去旧路径找配置。这个失效是**响的**（找不到会打印 `Failed to load config file` 并退出），不是静默的。

## 9. 附录：关键事实的源码依据

| 事实 | 依据 |
|---|---|
| Odin 发布 `odom → imu`、`imu → lidar`、`odom → camera_0`；`odom → map` 仅在重定位模式 | `src/odin_ros_driver/include/host_sdk_sample.h:1378, 1423, 1453, 1556` |
| **三条 TF 全部门控于 `send_odom_baselink_tf`，且 stamp 全部走 `make_aligned_stamp`（即受 `use_host_ros_time` 控制）** | `src/odin_ros_driver/include/host_sdk_sample.h:1372`（`if` 开块）、`1377`（`odom→imu` 的 stamp）、`1206`（`make_aligned_stamp` 调用）、`1422-1486`（`tf_til` / `tf_wc` 均在块内） |
| **`host_sdk_sample` 不读任何 ROS 参数**；配置路径由编译期 `__FILE__` 推出 `<源码树>/config/control_command.yaml` | `src/odin_ros_driver/src/host_sdk_sample.cpp:2491-2497`（组装路径）、`869-882`（`get_package_source_directory` 用 `__FILE__` 向上找 `package.xml`）；`grep 'get_parameter\|declare_parameter' src/host_sdk_sample.cpp` 零命中；`build/odin_ros_driver/compile_commands.json` 证实 `__FILE__` = `/home/shi/nav/src/odin_ros_driver/src/host_sdk_sample.cpp` |
| 驱动对配置键有白名单，白名单外的键会**报错**（`custom_*` 除外） | `src/odin_ros_driver/include/yaml_parser.h:104-165`（三个 `allowed_key_w_*_val` 集合，共 42 键） |
| 驱动会写 `src/odin_ros_driver/config/`（连接日志、calib 备份） | `src/odin_ros_driver/src/host_sdk_sample.cpp:1383-1400`、`1611-1612` |
| Odin 话题：`/odin1/odometry`（`odom` → `imu`）、`/odin1/imu`（frame `imu`）、`/odin1/cloud_slam`（frame `odom`）、`/odin1/cloud_raw`（frame `lidar`） | `src/odin_ros_driver/include/host_sdk_sample.h:1932-1944` |
| MID360 话题 `/livox/lidar`、`/livox/imu`；IMU 的 frame 硬编码为 `livox_frame`；驱动不发任何 TF | `src/livox_ros_driver2/src/lddc.cpp:481, 642-690` |
| MID360 时间戳在 NoSync 下取主机时间 | `src/livox_ros_driver2/src/comm/pub_handler.cpp:265-274` → `src/livox_ros_driver2/src/lddc.cpp:310` |
| MID360 配置无 timesync 段 | `src/livox_ros_driver2/config/MID360_config.json` |
| 轮半径 0.07617 m | `src/mysystem/meshes/wheel_1.stl` 包围盒 |
| 轮心在 base_link 上方 0.0762 m | `src/mysystem/urdf/robot.urdf` 的 `joint_wheel_*` origin |
| URDF 关节链与 origin | `src/mysystem/urdf/robot.urdf` 的 `joint_up`、`joint_imu`、`joint_livox_frame` |
| `mynav` 无 `install()` 规则 | `src/mynav/CMakeLists.txt` |
| 全仓库无 `/odom`、`/scan`、TF 生产者 | 排除 `src/libxr/` 与两个驱动后的全仓检索结果为空 |
| ~~Odin 主节点只接受 `config_file` 一个参数~~ **（已证伪）**上游 launch 确实传了 `config_file`，但节点从不读它 —— 该参数是装饰性的 | `src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py:55-57` vs `src/odin_ros_driver/src/host_sdk_sample.cpp:2491-2497` |
| livox launch 用相对路径 `../config/MID360_config.json` 指向配置文件 | `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py:17-19` |
| 两个驱动都把 `config/` 安装进 `share/` | `install/livox_ros_driver2/share/livox_ros_driver2/config/`、`install/odin_ros_driver/share/odin_ros_driver/config/` |
