#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$(readlink -f "$0")")"

ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ ! -f "$ROS_SETUP" ]; then
  ROS_SETUP="/opt/ros/jazzy/setup.bash"
fi
if [ ! -f "$ROS_SETUP" ]; then
  echo "[mybuild] 找不到 ROS 环境：$ROS_SETUP" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$ROS_SETUP"

# explore_lite_msgs 的 Python 生成目录与 --symlink-install 冲突，先用普通安装构建。
colcon build --packages-select explore_lite_msgs --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=OFF

# 两个驱动包含 Python 生成目录，也用普通安装方式构建。
colcon build --packages-select livox_ros_driver2 odin_ros_driver \
  --cmake-clean-cache \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_SYSTEM=ROS2 \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=jazzy

colcon build --symlink-install \
  --packages-skip explore_lite_msgs livox_ros_driver2 odin_ros_driver \
  --cmake-args \
    -DBUILD_TESTING=OFF \
    -DBUILD_SYSTEM=ROS2 \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=jazzy
