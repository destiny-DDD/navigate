# 感知层与 TF 底座 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 TF 树自洽、`odom → base_link` 可达、MID360 点云以 nav2 能消费的形式进入代价地图，并可在 Foxglove 中观察。

**Architecture:** 不写任何融合代码。Odin1 是唯一里程计源，它发布 `odom → imu`；URDF 的根已经改成 `imu`（已完成），于是 `imu → up → base_link` 由 `robot_state_publisher` 补上，两者拼成完整链。其余工作是把两个驱动的配置摆正（帧、点云格式、时间基准）、用一次启动时覆写把 Odin 配置落到驱动真正会读的路径上、清理 `nav2_params.yaml` 里指向不存在话题的引用，并用一把 bringup launch 把它们串起来。

**Tech Stack:** ROS 2 Jazzy、colcon / ament_cmake、Python 3（launch 与 pytest）、`tf2`、`robot_state_publisher`、`livox_ros_driver2`、`odin_ros_driver`、`foxglove_bridge`。

**Spec:** `docs/superpowers/specs/2026-09-17-perception-tf-base-design.md`（计划与 spec 冲突时以 spec 为准，并回来改计划）

## Global Constraints

以下约束适用于**每一个** task，不再逐条重复：

- **不要提交 git。** 用户明确指令「别传git」。本计划所有 task 的最后一步都是验证，**没有 commit 步骤**。改动留在工作区，由用户自行审核后决定。
- ROS 2 **Jazzy**；构建用本仓库根目录的 `./mybuild.sh`（即 `colcon build --packages-skip livox_ros_driver2 odin_ros_driver`）。它**会**构建 `mysystem` 与 `mynav`，**不会**构建两个驱动 —— 驱动已经构建过，本计划不需要重建它们。
- **唯一里程计源是 Odin1。** 本阶段不开 AMCL、不引入滤波器、不接轮速计。
- **树中没有 `map` 坐标系。** `global_frame` 类参数一律用 `odom`。
- **机器人本体坐标系统一为 `base_link`**，弃用 `base_footprint`。
- **不改动 vendored 仓库（`src/odin_ros_driver`、`src/livox_ros_driver2`）中的任何文件**，唯一例外是 §5.4 定义的「启动时覆写」：bringup 在启动前把我们的配置副本写到 `src/odin_ros_driver/config/control_command.yaml`。
- **不做 nav2 参数调优。** 只保证观测源接得上、坐标系不矛盾。
- **转盘（`joint_up`）本阶段保持 0 位、不动。** `joint_up` 保持 `continuous` 类型不变。
- 面向上游驱动的所有改动都通过 **launch 传参**，不编辑上游 launch 文件。

---

### Task 1: mynav 可安装 + URDF/TF 不变量回归测试

**为什么先做这个：** `src/mynav/CMakeLists.txt` 目前只有 `ament_package()`，没有任何 `install()` 规则 —— `config/` 从来就没被安装到 `share/` 过（已实测：`install/mynav/share/mynav/` 下没有 `config/`）。不修这个，后面所有配置都加载不了。同时把 URDF 的不变量钉成测试，锁住刚完成的 §5.3 改动。

**Files:**
- Modify: `src/mynav/CMakeLists.txt`
- Modify: `src/mynav/package.xml`
- Create: `src/mynav/test/test_urdf_invariants.py`

**前提：** spec §5.3 的 URDF 改动**已经做在工作区里但尚未提交**（根已改为 `imu`）。测试读的是 `install/` 里安装后的 URDF，所以 **Step 4 的 `./mybuild.sh` 是必需的** —— 不重新构建 `mysystem`，测的还是旧的 `base_link` 为根的版本，`test_root_is_imu` 会失败。这不是 bug，是刻意让"改了 URDF 忘记构建"暴露出来。

**Interfaces:**
- Consumes: `src/mysystem/urdf/robot.urdf`（§5.3 已完成，根已是 `imu`）
- Produces: `ament_add_pytest_test` 测试骨架，Task 2 与 Task 4 会在同一个 CMakeLists 块里追加各自的测试；`install(DIRECTORY config ...)` 规则，Task 2 的配置文件靠它才能被测试读到。

- [ ] **Step 1: 写失败的测试**

创建 `src/mynav/test/test_urdf_invariants.py`：

```python
#!/usr/bin/env python3
"""robot.urdf 的 TF 结构不变量与几何不变量。

本设计的整棵 TF 树建立在
    odom --(Odin 驱动)--> imu --(URDF)--> up --> base_link
之上。URDF 里的外参一旦被手改，ROS 不会报任何错，只会让传感器与底盘悄悄错位；
在 Foxglove 里表现为「点云有点偏」，极易被误判成标定或 nav2 参数问题。
这个测试把结构和外参钉成物理常量。

注意：读的是 **install/ 里安装后的** URDF，不是 src/ 下的源文件。
改了 URDF 必须重新 colcon build 才生效 —— 这是刻意的，测的是真正发出去的东西。
"""
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

URDF = os.path.join(get_package_share_directory("mysystem"), "urdf", "robot.urdf")

EXPECTED_ROOT = "imu"
EXPECTED_LINKS = {
    "imu", "up", "base_link", "livox_frame",
    "wheel_1", "wheel_2", "wheel_3", "wheel_4",
}

# 关节类型是承重的设计决定，不是随手写的：
#   joint_up 必须保持 continuous —— §4.3：将来接转盘时 URDF 不用改
#   joint_imu_up 必须 fixed —— 它只是把 URDF 的根从 base_link 挪到 imu
EXPECTED_JOINT_TYPES = {
    "joint_imu_up": "fixed",
    "joint_up": "continuous",
    "joint_livox_frame": "fixed",
    "joint_wheel_1": "continuous",
    "joint_wheel_2": "continuous",
    "joint_wheel_3": "continuous",
    "joint_wheel_4": "continuous",
}

# 重挂之后这两个关节的 rpy 必须为 0。§5.3 的数值自洽性论证
# （「逆变换即取负」）成立的前提就是它们没有旋转。
ZERO_RPY_JOINTS = ["joint_imu_up", "joint_up"]

# 期望外参：child 在 parent 下的平移，单位米。旋转关节按 0 角计。
EXPECTED_TRANSLATIONS = [
    ("imu", "up", (-0.0768, -0.025, -0.063)),
    ("up", "base_link", (0.0, 0.0, -0.1039)),
    ("imu", "base_link", (-0.0768, -0.025, -0.1669)),  # 整棵树的关键数字
    ("up", "livox_frame", (-0.0808, 0.0, 0.0951)),
    ("base_link", "livox_frame", (-0.0808, 0.0, 0.199)),
    ("base_link", "wheel_1", (0.11, 0.1605, 0.0762)),
    ("base_link", "wheel_2", (-0.11, 0.1605, 0.0762)),
    ("base_link", "wheel_3", (-0.11, -0.1605, 0.0762)),
    ("base_link", "wheel_4", (0.11, -0.1605, 0.0762)),
]

I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def rpy_to_R(r, p, y):
    """URDF 约定：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)。"""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_t(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def compose(T1, T2):
    """先施加 T2 再施加 T1：返回 (R1@R2 | R1@t2 + t1)。"""
    (R1, t1), (R2, t2) = T1, T2
    return (mat_mul(R1, R2), [a + b for a, b in zip(mat_vec(R1, t2), t1)])


def invert(T):
    R, t = T
    Rt = mat_t(R)
    return (Rt, [-v for v in mat_vec(Rt, t)])


def _vec(s):
    return [float(v) for v in s.split()]


def parse():
    """返回 (root_link, links, pose, joint_types, joint_rpy)。

    pose[link] = 该 link 在根坐标系下的位姿 (R, t)。
    """
    root = ET.parse(URDF).getroot()
    links = {l.get("name") for l in root.findall("link")}
    edges, joint_types, joint_rpy = {}, {}, {}
    for j in root.findall("joint"):
        name = j.get("name")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")
        assert child not in edges, (
            f"link '{child}' 有两个父关节：{edges[child][0]} 与 {parent}"
        )
        o = j.find("origin")
        xyz = _vec(o.get("xyz", "0 0 0")) if o is not None else [0.0] * 3
        rpy = _vec(o.get("rpy", "0 0 0")) if o is not None else [0.0] * 3
        edges[child] = (parent, (rpy_to_R(*rpy), xyz))
        joint_types[name] = j.get("type")
        joint_rpy[name] = rpy

    roots = links - set(edges)
    assert len(roots) == 1, f"期望恰好一个根 link，实际得到 {sorted(roots)}"
    root_link = roots.pop()

    children_of = {}
    for c, (p, _) in edges.items():
        children_of.setdefault(p, []).append(c)
    pose = {root_link: (I3, [0.0, 0.0, 0.0])}
    stack = [root_link]
    while stack:
        cur = stack.pop()
        for c in children_of.get(cur, []):
            pose[c] = compose(pose[cur], edges[c][1])
            stack.append(c)
    assert links == set(pose), f"这些 link 从根不可达：{sorted(links - set(pose))}"
    return root_link, links, pose, joint_types, joint_rpy


def rel(pose, a, b):
    """b 在 a 坐标系下的位姿。"""
    return compose(invert(pose[a]), pose[b])


def test_root_is_imu():
    root_link, _, _, _, _ = parse()
    assert root_link == EXPECTED_ROOT, (
        f"URDF 的根是 '{root_link}'，期望 '{EXPECTED_ROOT}'。"
        "换根是有意为之，见 spec §5.3 —— Odin 提供 odom→imu，URDF 必须接在 imu 下面。"
    )


def test_link_set():
    _, links, _, _, _ = parse()
    assert links == EXPECTED_LINKS, (
        f"多出 {sorted(links - EXPECTED_LINKS)}，缺少 {sorted(EXPECTED_LINKS - links)}"
    )


def test_joint_types():
    _, _, _, joint_types, _ = parse()
    assert joint_types == EXPECTED_JOINT_TYPES, (
        "关节类型变了。joint_up 必须是 continuous（§4.3：转盘将来要用）；"
        "改成 fixed 会让将来接转盘时必须再改一次 URDF。"
    )


def test_reroot_joints_have_zero_rpy():
    _, _, _, _, joint_rpy = parse()
    for name in ZERO_RPY_JOINTS:
        assert joint_rpy[name] == [0.0, 0.0, 0.0], (
            f"{name} 的 rpy 不再是 0（{joint_rpy[name]}）。"
            "§5.3 的全部数值论证都建立在「逆变换即取负」上，加旋转会让那条论证失效。"
        )


def test_pairwise_translations():
    """逐对断言外参平移 —— 本设计唯一具有真正回归价值的断言。"""
    _, _, pose, _, _ = parse()
    bad = []
    for a, b, expected in EXPECTED_TRANSLATIONS:
        _, t = rel(pose, a, b)
        if max(abs(x - y) for x, y in zip(t, expected)) > 1e-9:
            bad.append(f"{a} -> {b}: 期望 {list(expected)}，实际 {[round(v, 6) for v in t]}")
    assert not bad, "外参被改动了：\n" + "\n".join(bad)


def test_imu_to_base_link_is_pure_translation():
    """整棵树最关键的一跳：imu→base_link 必须是纯平移且为设计值。

    这一跳把 Odin 的 IMU 和底盘焊在一起；它一旦带上旋转，
    底盘朝向就会随 Odin 的世界系一起转，而 Foxglove 里看不出来。
    """
    _, _, pose, _, _ = parse()
    R, t = rel(pose, "imu", "base_link")
    assert max(abs(R[i][j] - I3[i][j]) for i in range(3) for j in range(3)) < 1e-12, (
        f"imu -> base_link 出现了旋转：{R}"
    )
    assert max(abs(x - y) for x, y in zip(t, (-0.0768, -0.025, -0.1669))) < 1e-9, (
        f"imu -> base_link 平移是 {[round(v, 6) for v in t]}"
    )
```

- [ ] **Step 2: 运行测试，确认它失败**

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest src/mynav/test/test_urdf_invariants.py -v
```

期望：**FAIL**，报 `ModuleNotFoundError: No module named 'ament_index_python'` 或 `mysystem not found`。

（若你先 `source install/setup.bash`，则可能已经能跑 —— 那就跳到 Step 3 的 CMake 集成，测试本身仍然要先确认在**未构建**状态下不被 ROS 感知。）

- [ ] **Step 3: 接上 CMake 与依赖**

修改 `src/mynav/CMakeLists.txt`，在 `find_package(rclcpp REQUIRED)` 之后、`if(BUILD_TESTING)` 之前插入 install 规则：

```cmake
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```

然后替换整个 `if(BUILD_TESTING)` 块为：

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(urdf_invariants test/test_urdf_invariants.py)
endif()
```

> **注意：** 这里**只**安装 `config`。`launch` 目录要到 Task 3 才存在，而 `install(DIRECTORY launch ...)` 在目录不存在时会让 CMake 直接报错。

修改 `src/mynav/package.xml`，在 `<depend>rclcpp</depend>` 之后追加：

```xml
  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>mysystem</test_depend>
```

- [ ] **Step 4: 构建并运行测试，确认通过**

```bash
cd /home/shi/nav
./mybuild.sh
source install/setup.bash
colcon test --packages-select mynav --event-handlers console_direct+
```

期望：`urdf_invariants` **PASS**（6 个测试）。

同时确认配置终于被安装了：

```bash
ls install/mynav/share/mynav/config/nav2_params.yaml
```

期望：文件存在。**在做这个 task 之前该路径不存在** —— 这就是 §5.8 关键点二说的那个 bug。

- [ ] **Step 5: 确认改动范围，不要提交**

```bash
git status --short
```

期望：`M src/mynav/CMakeLists.txt`、`M src/mynav/package.xml`、`?? src/mynav/test/`，没有别的。**不要 commit**（Global Constraints）。

---

### Task 2: Odin 配置副本 + 完整性测试

**背景（必读）：** `host_sdk_sample` **不读任何 ROS 参数**。它的配置路径由编译期 `__FILE__` 推出，即 `src/odin_ros_driver/config/control_command.yaml`，运行时无法改变（依据见 spec §9）。所以我们的副本只有在 bringup 把它**覆写回去**之后才真正生效 —— 覆写本身在 Task 3。这个 task 只负责把副本做出来并守住它的完整性。

**Files:**
- Create: `src/mynav/config/odin_control_command.yaml`（由上游复制而来）
- Create: `src/mynav/test/test_odin_config.py`
- Modify: `src/mynav/CMakeLists.txt`（追加一行测试注册）

**Interfaces:**
- Consumes: Task 1 的 `install(DIRECTORY config ...)` 规则与 `ament_add_pytest_test` 骨架
- Produces: `install/mynav/share/mynav/config/odin_control_command.yaml`，Task 3 的 `bringup.launch.py` 会把这个已安装的副本覆写到驱动真正读取的路径上。

- [ ] **Step 1: 复制上游配置**

```bash
cd /home/shi/nav
cp src/odin_ros_driver/config/control_command.yaml src/mynav/config/odin_control_command.yaml
```

- [ ] **Step 2: 改时间基准**

在 `src/mynav/config/odin_control_command.yaml` 中，把第 9 行改成：

```yaml
  use_host_ros_time: 1  # MID360 的时间戳是主机时间（NoSync），Odin 必须同源；否则代价地图查 TF 外插失败、点云被静默丢弃
```

**只改这一行。** 其余 43 个键的**值**保持与上游一致。

- [ ] **Step 3: 写完整性测试**

创建 `src/mynav/test/test_odin_config.py`：

```python
#!/usr/bin/env python3
"""Odin 配置副本的完整性检查。

背景：host_sdk_sample **不读任何 ROS 参数**，配置路径由编译期 __FILE__ 推出，
即 src/odin_ros_driver/config/control_command.yaml。所以 mynav 里这份副本
只有在 bringup 把它覆写回去之后才真正生效（见 bringup.launch.py）。

这个测试守两件事：
  1. use_host_ros_time == 1 —— 全设计里最容易静默失效的一环（spec §5.9）；
  2. 副本没有在编辑中掉键 —— 该 yaml 被整体读入当参数表，缺键会静默退回
     代码内默认值，是第二难察觉的失效（spec §6.5）。
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory

ODIN_CFG = os.path.join(
    get_package_share_directory("mynav"), "config", "odin_control_command.yaml"
)

# 冻结清单：2026-09-17 上游 control_command.yaml 的 44 个键。
# 刻意不写成「与上游文件比对」：§5.4 的覆写机制会把 vendored 文件变成我们
# 自己的副本，比对对象会自我指涉。冻结清单没有这个问题。
FROZEN_KEYS = {
    "strict_usb3.0_check", "use_host_ros_time", "streamctrl",
    "sendrgbcompressed", "sendrgb", "sendrgbundistort", "sendimu",
    "enable_imu_smooth", "imu_smooth_frequency", "sendodom",
    "send_odom_baselink_tf", "tf_extra_publish_rate", "senddtof",
    "cloud_raw_confidence_threshold", "dtof_fps", "rgb_format", "rgb_width",
    "rgb_height", "rgb_fps", "sendcloudslam", "sendcloudrender",
    "senddepth", "sendreprojection", "sendoverlay",
    "overlay_reprojected_topic", "overlay_camera_topic",
    "overlay_output_topic", "overlay_alpha", "recorddata", "devstatuslog",
    "save_log", "pubintensitygray", "showpath", "showcamerapose",
    "custom_map_mode", "custom_init_pos", "custom_init_pose_search_radius",
    "custom_init_pose_max_rot_deg", "relocalization_map_abs_path",
    "mapping_result_dest_dir", "mapping_result_file_name", "sendimagemask",
    "image_mask_abs_path", "resetalgo",
}


def _keys():
    with open(ODIN_CFG) as f:
        return yaml.safe_load(f)["register_keys"]


def test_use_host_ros_time_is_one():
    got = _keys().get("use_host_ros_time")
    assert got == 1, (
        f"use_host_ros_time 是 {got!r}，必须是 1。"
        "MID360 在 NoSync 下用主机时间戳；Odin 用设备时间戳的话两者不同源，"
        "代价地图会拿点云时间去查 TF、外插失败、静默丢点云 —— "
        "表现是「点云进不了代价地图」，极容易被误判成 nav2 参数问题。"
    )


def test_no_key_was_dropped():
    missing = FROZEN_KEYS - set(_keys())
    assert not missing, (
        f"这些键从配置副本里消失了：{sorted(missing)}。"
        "该 yaml 被整体读入当参数表，缺键不会报错，只会静默退回代码内默认值。"
    )


def test_odometry_and_tf_stay_enabled():
    """这两个开关是整棵 TF 树的入口，关掉就等于没有 odom→imu。"""
    keys = _keys()
    assert keys["sendodom"] == 1
    assert keys["send_odom_baselink_tf"] == 1, (
        "send_odom_baselink_tf 门控的不只是 odom→imu，"
        "imu→lidar 与 odom→camera_0 也嵌在同一个 if 块里（spec §8 风险 4）。"
    )
```

- [ ] **Step 4: 注册测试**

在 `src/mynav/CMakeLists.txt` 的 `if(BUILD_TESTING)` 块中，`ament_add_pytest_test(urdf_invariants ...)` 之后追加：

```cmake
  ament_add_pytest_test(odin_config test/test_odin_config.py)
```

在 `src/mynav/package.xml` 的 `<test_depend>python3-pytest</test_depend>` 之后追加：

```xml
  <test_depend>python3-yaml</test_depend>
```

- [ ] **Step 5: 构建并运行，确认通过**

```bash
cd /home/shi/nav
./mybuild.sh
source install/setup.bash
colcon test --packages-select mynav --event-handlers console_direct+
```

期望：`odin_config` 3 个测试 **PASS**。

- [ ] **Step 6: 确认改动范围，不要提交**

```bash
git status --short
```

期望：新增 `src/mynav/config/odin_control_command.yaml` 与 `src/mynav/test/test_odin_config.py`，以及 Task 1 的改动。**不要 commit**。

---

### Task 3: bringup launch（含 Odin 配置覆写）

**Files:**
- Create: `src/mynav/launch/bringup.launch.py`
- Modify: `src/mynav/CMakeLists.txt`（install 列表加上 `launch`）
- Modify: `src/mynav/package.xml`（launch 期依赖）

**Interfaces:**
- Consumes: Task 2 的 `install/mynav/share/mynav/config/odin_control_command.yaml`；`mysystem` 的 `launch/mysystem_love_launch.py`（已存在，提供 RSP + JSP + Foxglove）
- Produces: 可运行的 `ros2 launch mynav bringup.launch.py`，含 launch 参数 `odin_config_dest` / `start_lidar` / `start_odin` / `use_foxglove` / `use_gui`。Task 5 的验收跑它。

- [ ] **Step 1: 写 launch 文件**

创建 `src/mynav/launch/bringup.launch.py`：

```python
#!/usr/bin/env python3
"""感知层 bringup：MID360 + Odin1 + RSP/JSP + Foxglove。

不启动 nav2 —— 本阶段只交付感知与 TF 底座（spec §3）。

TF 树的目标形态（spec §5.2）：
    odom
    ├─ imu            ← Odin 驱动发布 odom→imu
    │   ├─ lidar      ← Odin 驱动发布 imu→lidar（需等设备回传标定后才出现）
    │   └─ up         ← 以下由 robot_state_publisher 从 URDF 发布
    │       ├─ base_link
    │       │   └─ wheel_1..4
    │       └─ livox_frame
    └─ camera_0       ← Odin 驱动发布 odom→camera_0（同上）
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ODIN_CONFIG_RELPATH = os.path.join("src", "odin_ros_driver", "config",
                                   "control_command.yaml")


def _odin_source_config_path():
    """推出 host_sdk_sample 实际会读的配置路径。

    该路径在驱动里是编译期用 __FILE__ 向上找 package.xml 得到的
    （host_sdk_sample.cpp:869-882, 2491-2497），运行时无法改变。
    我们在 launch 期能拿到的最接近的近似是用 COLCON_PREFIX_PATH 推工作区根 ——
    这正是驱动自己在别处用的办法（host_sdk_sample.cpp:1349）。

    推不出来时返回 None，由 _overlay_odin_config 明确报错，
    而不是静默用一个错误路径。
    """
    prefix = os.environ.get("COLCON_PREFIX_PATH", "")
    idx = prefix.find("/install")
    if idx == -1:
        return None
    return os.path.join(prefix[:idx], ODIN_CONFIG_RELPATH)


def _overlay_odin_config(context, *args, **kwargs):
    """把我们的 Odin 配置副本覆写到驱动真正会读的那个路径上。

    为什么必须这么做：host_sdk_sample 不读任何 ROS 参数，配置路径在编译期就
    焊死了（spec §5.4）。真相源仍然是 mynav 里的副本；vendored 文件只是一个
    每次启动重新施加的输出 —— 重新拉取覆盖了它也不会静默生效。
    """
    if context.launch_configurations.get("start_odin", "true").lower() != "true":
        return []

    dest = context.launch_configurations.get("odin_config_dest", "")
    src = os.path.join(get_package_share_directory("mynav"), "config",
                       "odin_control_command.yaml")

    if not dest:
        raise RuntimeError(
            "无法推出 Odin 配置文件的目标路径（COLCON_PREFIX_PATH 里没有 /install）。\n"
            "请显式传入，例如：\n"
            "  ros2 launch mynav bringup.launch.py "
            "odin_config_dest:=/home/shi/nav/src/odin_ros_driver/config/control_command.yaml"
        )

    dest_dir = os.path.dirname(dest)
    if not os.path.isdir(dest_dir):
        raise RuntimeError(
            f"Odin 配置目录不存在：{dest_dir}\n"
            "host_sdk_sample 的配置路径由编译期 __FILE__ 决定，必须是源码树里的 "
            "src/odin_ros_driver/config/。若你在只有 install/ 的机器上启动，"
            "请用 odin_config_dest:=<绝对路径> 显式指定。"
        )

    shutil.copyfile(src, dest)
    return [LogInfo(msg=f"[bringup] Odin 配置已覆写：{src} -> {dest}")]


def generate_launch_description():
    livox_share = get_package_share_directory("livox_ros_driver2")
    mysystem_share = get_package_share_directory("mysystem")

    odin_config_dest = DeclareLaunchArgument(
        "odin_config_dest",
        default_value=_odin_source_config_path() or "",
        description="host_sdk_sample 实际读取的 control_command.yaml 绝对路径",
    )
    start_lidar = DeclareLaunchArgument(
        "start_lidar", default_value="true",
        description="启动 MID360 驱动",
    )
    start_odin = DeclareLaunchArgument(
        "start_odin", default_value="true",
        description="启动 Odin1 驱动（并执行配置覆写）",
    )
    use_foxglove = DeclareLaunchArgument(
        "use_foxglove", default_value="true",
        description="启动 Foxglove Bridge",
    )
    use_gui = DeclareLaunchArgument(
        "use_gui", default_value="false",
        description="用 joint_state_publisher_gui 手动发布关节角",
    )

    # MID360：直接给节点传参，不用也不改上游的 msg_MID360_launch.py（spec §5.0/§5.5）
    livox = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_lidar")),
        parameters=[{
            "xfer_format": 0,        # 0 = PointCloud2。nav2 代价地图吃不了 CustomMsg
            "multi_topic": 0,        # 单雷达，话题 /livox/lidar
            "data_src": 0,           # 0 = 雷达
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",   # 与 URDF 里的 livox_frame 一致
            "lvx_file_path": "",         # data_src=0 时不使用
            # 必须是**安装后**的绝对路径：上游用相对 launch 文件的路径，
            # 我们自己的 launch 不在那个目录下（spec §5.5）
            "user_config_path": os.path.join(livox_share, "config",
                                             "MID360_config.json"),
            "cmdline_input_bd_code": "livox0000000001",
        }],
    )

    # Odin1：直接起 host_sdk_sample，不用上游 launch（spec §5.8 关键点一）。
    # 不传任何参数 —— 该节点不读 ROS 参数，配置靠上面的覆写。
    # 重映射只挂在驱动节点上：上游 launch 里的 cloud_reprojection_ros2_node
    # 会订阅 /odin1/odometry，重映射后它收不到数据，所以不启动那些辅助节点。
    odin = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_odin")),
        remappings=[("/odin1/odometry", "/odom")],
    )

    # RSP + JSP + Foxglove 复用 mysystem 已有的 launch，不重复造
    rsp_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mysystem_share, "launch", "mysystem_love_launch.py")
        ),
        launch_arguments={
            "use_gui": LaunchConfiguration("use_gui"),
            "use_foxglove": LaunchConfiguration("use_foxglove"),
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(odin_config_dest)
    ld.add_action(start_lidar)
    ld.add_action(start_odin)
    ld.add_action(use_foxglove)
    ld.add_action(use_gui)
    # 覆写必须早于 odin 节点启动
    ld.add_action(OpaqueFunction(function=_overlay_odin_config))
    ld.add_action(livox)
    ld.add_action(odin)
    ld.add_action(rsp_stack)
    return ld
```

- [ ] **Step 2: 把 launch 目录纳入安装**

修改 `src/mynav/CMakeLists.txt` 的 install 块（`launch` 目录现在存在了）：

```cmake
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
```

修改 `src/mynav/package.xml`，在 `<depend>rclcpp</depend>` 之后追加：

```xml
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>mysystem</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>foxglove_bridge</exec_depend>
  <exec_depend>livox_ros_driver2</exec_depend>
  <exec_depend>odin_ros_driver</exec_depend>
```

- [ ] **Step 3: 构建并确认 launch 被安装**

```bash
cd /home/shi/nav
./mybuild.sh
ls install/mynav/share/mynav/launch/bringup.launch.py
```

期望：文件存在。

- [ ] **Step 4: 无硬件验证 —— 只起 URDF 那半边树**

这一半不需要任何硬件，因为 `imu → up → base_link → wheel_*` 和 `imu → up → livox_frame` 完全由 `robot_state_publisher` 从 URDF 发布。

```bash
source install/setup.bash
ros2 launch mynav bringup.launch.py start_lidar:=false start_odin:=false
```

在另一个终端：

```bash
source install/setup.bash
ros2 run tf2_tools view_frames
```

期望：生成 `frames.pdf` / `frames.gv`，树形是

```
imu
└─ up
   ├─ base_link
   │  ├─ wheel_1
   │  ├─ wheel_2
   │  ├─ wheel_3
   │  └─ wheel_4
   └─ livox_frame
```

即 `imu` 是**根**（没有 `odom`，因为 Odin 没启动；没有 `map`，本阶段就没有 `map`），无多父、无环。

再确认 `joint_up` 这条动态边是活的（spec §4.3 把它列为验收项）：

```bash
ros2 topic hz /joint_states
ros2 topic echo /tf --once     # 应能看到 imu -> up
```

期望：`/joint_states` 持续有输出；`/tf` 上有 `imu → up`（`joint_up` 是 `continuous`，走 `/tf` 而非 `/tf_static`）。

- [ ] **Step 5: 无硬件验证 —— 覆写确实发生且可失败**

```bash
# 先确认覆写真的改动了 vendored 文件
git diff --stat src/odin_ros_driver/config/control_command.yaml
```

期望：显示 1 处改动（`use_host_ros_time` 那行）。然后确认**失败是响的**：

```bash
ros2 launch mynav bringup.launch.py start_lidar:=false \
  odin_config_dest:=/nonexistent/dir/control_command.yaml
```

期望：启动**立即报错退出**，报 `Odin 配置目录不存在`，不是静默继续。

最后把这个副作用撤回，保持工作区干净：

```bash
git checkout -- src/odin_ros_driver/config/control_command.yaml
```

- [ ] **Step 6: 确认改动范围，不要提交**

```bash
git status --short
```

期望：`launch/bringup.launch.py` 为新增，CMakeLists / package.xml 被修改，vendored 配置**没有**未提交改动（Step 5 已撤回）。**不要 commit**。

---

### Task 4: `nav2_params.yaml` 清理

**范围限定：** 只做 spec §5.7 的「最小正确改动」—— 让观测源接得上、坐标系不矛盾。**不做任何参数调优**（分辨率、膨胀半径、规划器等一律不动）。

**Files:**
- Modify: `src/mynav/config/nav2_params.yaml`
- Create: `src/mynav/test/test_nav2_params.py`
- Modify: `src/mynav/CMakeLists.txt`（追加一行测试注册）

**Interfaces:**
- Consumes: Task 3 产出的话题名 `/livox/lidar`（`xfer_format: 0` 时的 PointCloud2 话题）
- Produces: 一份不含 `base_footprint`、不含 `/scan` 死引用、代价地图观测源指向 `/livox/lidar` 的参数文件。本阶段不启动 nav2，这份文件是给下一阶段准备的。

- [ ] **Step 1: 改局部代价地图的观测源**

在 `local_costmap.local_costmap.ros__parameters.voxel_layer` 下，把 `observation_sources: scan` 改为：

```yaml
        observation_sources: livox  # 障碍物观测来源名称。
```

把紧跟的整个 `scan:` 块替换为（块名一并改成 `livox`）：

```yaml
        livox:  # 体素层使用的 MID360 点云观测配置段。
          topic: /livox/lidar  # MID360 点云话题。
          max_obstacle_height: 2.0  # 该传感器可标记的最高障碍物，单位米。
          min_obstacle_height: 0.0  # 该传感器可标记的最低障碍物，单位米。
          clearing: True  # 空旷方向是否清除旧障碍物。
          marking: True  # 检测到障碍物时是否标记障碍物。
          data_type: "PointCloud2"  # 输入消息类型；nav2 吃不了 Livox 的 CustomMsg。
          raytrace_max_range: 3.0  # 清障射线的最大距离，单位米。
          raytrace_min_range: 0.0  # 清障射线的最小距离，单位米。
          obstacle_max_range: 2.5  # 标记障碍物的最大距离，单位米。
          obstacle_min_range: 0.0  # 标记障碍物的最小距离，单位米。
```

> 取值沿用 nav2 官方默认，**这是语义正确的起点，不是调优结果**（spec §5.7.1）。

- [ ] **Step 2: 改全局代价地图的观测源**

在 `global_costmap.global_costmap.ros__parameters.obstacle_layer` 下把 `observation_sources: scan` 改为：

```yaml
        observation_sources: livox  # 障碍物观测来源名称。
```

把整个 `scan:` 块替换为：

```yaml
        livox:  # 障碍物层使用的 MID360 点云观测配置段。
          topic: /livox/lidar  # MID360 点云话题。
          max_obstacle_height: 2.0  # 最高考虑的障碍物高度，单位米。
          min_obstacle_height: 0.0  # 最低考虑的障碍物高度，单位米。
          clearing: True  # 空旷方向是否清除旧障碍物。
          marking: True  # 检测到障碍物时是否标记障碍物。
          data_type: "PointCloud2"  # 输入消息类型；nav2 吃不了 Livox 的 CustomMsg。
          raytrace_max_range: 3.0  # 清障射线的最大距离，单位米。
          raytrace_min_range: 0.0  # 清障射线的最小距离，单位米。
          obstacle_max_range: 2.5  # 标记障碍物的最大距离，单位米。
          obstacle_min_range: 0.0  # 标记障碍物的最小距离，单位米。
```

- [ ] **Step 3: 改全局坐标系**

本阶段树中没有 `map`（spec §5.2），凡是会真正启动的节点都不能用 `map`：

| 位置 | 原值 | 改为 |
|---|---|---|
| `global_costmap.global_costmap.ros__parameters.global_frame` | `map` | `odom` |
| `bt_navigator.ros__parameters.global_frame` | `map` | `odom` |
| `behavior_server.ros__parameters.global_frame` | `map` | `odom` |

> **这条超出了 spec §5.7 的字面清单**（§5.7 第 2 项只点了全局代价地图）。加进来是因为 `bt_navigator` 与 `behavior_server` 都是下一阶段真正会启动的节点，而 §5.2 已确立本阶段不存在 `map` 坐标系 —— 留着它们等于保证下一阶段一启动就失败。**如果你不同意，改回 `map` 即可**，不影响本阶段任何验收项。

**保持 `map` 不动的两处**（本阶段明确不启动，属于重定位/仿真路径）：

- `amcl.ros__parameters.global_frame_id: "map"` —— §5.7.5 说 AMCL 保留但不启动，将来重定位时可能复用。
- `loopback_simulator.ros__parameters.map_frame_id: "map"` —— 仿真器，本阶段不用。

- [ ] **Step 4: 清理 `base_footprint` 与死引用**

| 位置 | 改动 |
|---|---|
| `amcl.ros__parameters.base_frame_id` | `"base_footprint"` → `"base_link"` |
| `amcl.ros__parameters.scan_topic` | **整行删除**（死引用：全仓没有 `/scan` 的生产者） |
| `collision_monitor.ros__parameters.base_frame_id` | `"base_footprint"` → `"base_link"` |
| `collision_monitor.ros__parameters.observation_sources` | `["scan"]` → `[]` |
| `collision_monitor.ros__parameters` 下的 `scan:` 块 | **整块删除**（含 `type` / `topic` / `min_height` / `max_height` / `enabled`） |
| `loopback_simulator.ros__parameters.base_frame_id` | `"base_footprint"` → `"base_link"` |
| `loopback_simulator.ros__parameters.scan_frame_id` | **整行删除** |

> `collision_monitor` 本阶段不启动。把它的观测源清空而不是改接到点云，是因为 nav2 的 collision_monitor 接点云需要 `type: "pointcloud"` 与一套不同的高度/时间参数，那属于调优范围。接它进来是后续工作。

**不要改** `velocity_smoother.ros__parameters.odom_topic: "odom"` —— 它已经和 Task 3 的重映射目标一致。

- [ ] **Step 5: 写参数清理的回归测试**

创建 `src/mynav/test/test_nav2_params.py`：

```python
#!/usr/bin/env python3
"""nav2_params.yaml 的坐标系一致性检查。

本阶段树中没有 map 坐标系，机器人本体坐标系是 base_link（spec §5.1/§5.2）。
这组断言防止清理过的引用被悄悄改回去 —— 这类错误不会在启动时报错，
只会让某个节点在运行时静默失联。
"""
import os

import yaml

from ament_index_python.packages import get_package_share_directory

PARAMS = os.path.join(get_package_share_directory("mynav"), "config",
                      "nav2_params.yaml")


def _params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def test_no_base_footprint_anywhere():
    with open(PARAMS) as f:
        raw = f.read()
    assert "base_footprint" not in raw, (
        "本设计已弃用 base_footprint（spec §4.4：base_link 的接地点就在 z=0，"
        "base_footprint 会是恒等变换）。统一用 base_link。"
    )


def test_no_scan_topic_references():
    """全仓没有 /scan 的生产者，任何指向它的配置都是死引用。"""
    with open(PARAMS) as f:
        raw = f.read()
    for dead in ('topic: /scan', 'topic: "scan"', 'topic: scan',
                 'scan_topic: scan', 'scan_frame_id'):
        assert dead not in raw, f"仍有死引用：{dead!r}"


def test_costmaps_observe_livox_pointcloud():
    p = _params()
    local = p["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    glob = p["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]
    for name, layer in (("local", local), ("global", glob)):
        assert layer["observation_sources"] == "livox", f"{name} 的观测源不是 livox"
        src = layer["livox"]
        assert src["topic"] == "/livox/lidar"
        assert src["data_type"] == "PointCloud2", (
            "MID360 在 xfer_format=0 下发布 PointCloud2；CustomMsg 代价地图吃不了"
        )
        assert src["marking"] is True and src["clearing"] is True


def test_global_frames_are_odom_not_map():
    p = _params()
    assert p["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] == "odom"
    assert p["bt_navigator"]["ros__parameters"]["global_frame"] == "odom"
    assert p["behavior_server"]["ros__parameters"]["global_frame"] == "odom"


def test_robot_base_frame_is_base_link():
    p = _params()
    assert p["local_costmap"]["local_costmap"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert p["global_costmap"]["global_costmap"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert p["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert p["behavior_server"]["ros__parameters"]["robot_base_frame"] == "base_link"
```

- [ ] **Step 6: 注册测试、构建、运行**

在 `src/mynav/CMakeLists.txt` 中追加：

```cmake
  ament_add_pytest_test(nav2_params test/test_nav2_params.py)
```

```bash
cd /home/shi/nav
./mybuild.sh
source install/setup.bash
colcon test --packages-select mynav --event-handlers console_direct+
```

期望：`nav2_params` 5 个测试 **PASS**，Task 1/2 的测试仍然 PASS。

- [ ] **Step 7: 确认改动范围，不要提交**

```bash
git status --short
```

期望：`M src/mynav/config/nav2_params.yaml` 加新增测试文件。**不要 commit**。

---

### Task 5: 验收手册（`src/mynav/README.md`）

**为什么需要独立一个 task：** spec §6 的测试策略里，第 3、4 条（`/odom` 与 TF 一致性、时间戳同源）和第 7 章的验收标准**都需要真实硬件**，没法自动化。把它们写成一个可照着做的清单，是这个阶段能交付的最后一个东西。

**Files:**
- Create: `src/mynav/README.md`

**Interfaces:**
- Consumes: Task 1–4 的全部产出（`ros2 launch mynav bringup.launch.py` 与三组测试）
- Produces: 一份人可执行的验收流程，覆盖 spec §7。

- [ ] **Step 1: 写 README**

创建 `src/mynav/README.md`，内容至少覆盖：

1. **这个包是什么** —— 与 `mysystem`（描述层）的分工：`mysystem` 有 URDF/mesh/rviz，`mynav` 有 nav 相关配置与 bringup。
2. **构建** —— `./mybuild.sh`；提醒 `mybuild.sh` **不构建**两个驱动。
3. **跑测试** —— `colcon test --packages-select mynav --event-handlers console_direct+`，并说明三组测试各守什么。
4. **启动** —— `ros2 launch mynav bringup.launch.py`，列出四个 launch 参数的含义；给出无硬件时的用法（`start_lidar:=false start_odin:=false`）。
5. **`src/odin_ros_driver/config/control_command.yaml` 会被覆写** —— 说明真相源是 `mynav/config/odin_control_command.yaml`，vendored 文件只是每次启动重新施加的输出；看到 `git status` 里它变脏是正常的；用 `git checkout --` 撤回不影响下次启动。

   这一条要写清楚，否则将来有人会把它当成"有人手改了 vendored 文件"而去修一个不存在的问题。
6. **验收清单**（逐条抄 spec §7，并给出执行命令）：
   - `tf2_tools view_frames` 树形与 spec §5.2 一致（`odom` 为根、无多父、无环）
   - `odom → base_link` 可用，机器人移动时在 Foxglove 里正确跟随（**前提：托架停在 0 位**）
   - MID360 点云以 `PointCloud2` 发布，在 Foxglove 里与 TF 树贴合
   - 点云进入 nav2 代价地图，Foxglove 中可见障碍（不要求调优后的质量）
   - `/odom` 有数据
   - **链路方向验证（不可省略）：** 手动往 `/joint_states` 发一个已知的 `joint_up` 角度，断言 Foxglove 里 `base_link` 的朝向**确实随之偏转**。这是将来做转盘上报时唯一的地基。

     > 这是 §7 第 6 项，spec 明确标注不可省略。最容易的触发方式是用 `use_gui:=true` 起 bringup，拖 `joint_up` 的滑块。
   - `ros2 topic hz /joint_states` 持续有输出（保证 `imu → up` 这条动态边的存活）
   - **时间戳同源：** `ros2 topic echo /livox/lidar --field header.stamp` 与 `ros2 topic echo /tf --field transforms[0].header.stamp` 的数值应落在同一量级（都是主机时间的纳秒纪年），偏差在毫秒级而不是天级。

     这一条是 spec §5.9 说的"最容易静默失败的一环"，值得单独列。
7. **排查：树里没有 `lidar` / `camera_0`** —— 这两条边由 Odin 在收到设备回传的标定之后才发布（`has_cached_extrinsics_`），是启动暂态。等几秒再看。

- [ ] **Step 2: 校对每一条都能照着执行**

逐条走一遍 README 里给的命令，确认：
- 每条命令都是**可直接复制粘贴**的（含 `source install/setup.bash` 这类前置）
- 每条都写了**期望看到什么** —— 没有"检查 TF 是否正确"这种没法执行的句子
- spec §7 的 7 个验收项**一条不漏**

- [ ] **Step 3: 确认改动范围，不要提交**

```bash
git status --short
```

期望：新增 `src/mynav/README.md`。**不要 commit**。

---

## 完成之后

本计划交付的是 spec §3「在范围内」的全部内容。以下**刻意不在本计划内**（spec §3 已明确排除）：

- nav2 参数调优（分辨率、膨胀半径、MPPI 参数等）
- 启动 nav2 本身（`nav2_params.yaml` 清理好了，但本阶段不拉起 nav2）
- AMCL 与 `map` 坐标系（等重定位模式）
- 转盘角度上报（等后续独立工作；注意 spec §4.3 的符号约定提示与 §8 风险 5 的"锁定"限制）
- RGB 图像链路
- 轮速计

**下一阶段启动 nav2 时的第一件事：** 注意 `amcl.global_frame_id` 与 `loopback_simulator.map_frame_id` 仍然是 `map`（本阶段刻意保留），届时要么给树补上 `map`，要么把它们改掉。
