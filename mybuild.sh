#!/usr/bin/env bash
set -e

# explore_lite_msgs 的 Python 生成目录与 --symlink-install 冲突，先用普通安装构建。
colcon build --packages-select explore_lite_msgs

# 两个驱动包含 Python 生成目录，也用普通安装方式构建。
colcon build --packages-select livox_ros_driver2 odin_ros_driver \
  --cmake-args \
    -DBUILD_SYSTEM=ROS2 \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=jazzy

colcon build --symlink-install \
  --packages-up-to mynav my_bt \
  --packages-skip explore_lite_msgs livox_ros_driver2 odin_ros_driver \
  --cmake-args \
    -DBUILD_SYSTEM=ROS2 \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS=jazzy
