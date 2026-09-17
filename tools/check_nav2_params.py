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
    # --- collision_monitor 安全区 ---
    ("collision_monitor.polygons", CM + ("polygons",), ["StopZone", "SlowZone", "FootprintApproach"]),
    ("StopZone.action_type 为 stop", CM + ("StopZone", "action_type"), "stop"),
    ("SlowZone.action_type 为 slowdown", CM + ("SlowZone", "action_type"), "slowdown"),
    ("stop_pub_timeout 放宽到 5 秒", CM + ("stop_pub_timeout",), 5.0),
    # --- scan 源上的死参数已清除（Scan 类不读高度，见 pointcloud.hpp 对比）---
    ("scan 源已移除死参数 min_height", CM + ("scan", "min_height"), None),
    ("scan 源已移除死参数 max_height", CM + ("scan", "max_height"), None),
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
