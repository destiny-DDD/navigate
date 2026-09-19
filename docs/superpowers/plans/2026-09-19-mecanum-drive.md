# Mecanum Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Make the Gazebo Sim robot accept ROS `/cmd_vel`, publish ROS `/odom`, and broadcast `odom -> imu`.

**Architecture:** Keep `robot.urdf` as the only robot description and configure Gazebo Sim's `MecanumDrive` plugin in its Gazebo extension. The simulation launch starts the existing simulator and spawner, then starts a Gazebo-to-ROS bridge and a focused Python transform broadcaster.

**Tech Stack:** ROS 2 Jazzy, Gazebo Sim 8, `ros_gz_bridge`, `rclpy`, `tf2_ros`, `nav_msgs`.

**Spec:** `docs/superpowers/specs/2026-09-19-mecanum-drive-design.md`

## Global Constraints

- Use the existing `robot.urdf`; do not create a second SDF robot description.
- Use `/cmd_vel`, `/odom`, `/clock`, and the dynamic transform `odom -> imu`.
- Configure wheel radius `0.07618`, separation `0.321`, and wheelbase `0.22` in meters.
- Follow `src/mysystem/launch/mysystem_love_launch.py` formatting for launch files.
- Do not create commits, tags, remotes, pushes, or uploads.

---

### Task 1: Mecanum Plugin And Package Dependencies

**Files:**
- Modify: `src/mysystem/urdf/robot.urdf`
- Modify: `src/mysystem/package.xml`
- Test: `src/mysystem/test/test_sim_world_assets.py`

**Interfaces:**
- Produces: Gazebo model topics `/model/robot/cmd_vel` and `/model/robot/odometry`.
- Consumes: Existing joints `joint_wheel_1`, `joint_wheel_4`, `joint_wheel_2`, and `joint_wheel_3`.

- [x] **Step 1: Write failing structural tests**

```python
plugin = ET.parse(URDF_PATH).getroot().find("gazebo/plugin")
self.assertEqual(plugin.get("filename"), "gz-sim-mecanum-drive-system")
self.assertEqual(plugin.findtext("front_left_joint"), "joint_wheel_1")
self.assertEqual(plugin.findtext("front_right_joint"), "joint_wheel_4")
self.assertEqual(plugin.findtext("back_left_joint"), "joint_wheel_2")
self.assertEqual(plugin.findtext("back_right_joint"), "joint_wheel_3")
```

- [x] **Step 2: Run the test to verify it fails**

Run: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 src/mysystem/test/test_sim_world_assets.py`

Expected: the plugin lookup is `None` because the URDF has no Gazebo drive plugin.

- [x] **Step 3: Add the URDF plugin and runtime dependencies**

Add a root-level `<gazebo>` extension containing:

```xml
<plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
  <front_left_joint>joint_wheel_1</front_left_joint>
  <front_right_joint>joint_wheel_4</front_right_joint>
  <back_left_joint>joint_wheel_2</back_left_joint>
  <back_right_joint>joint_wheel_3</back_right_joint>
  <wheel_separation>0.321</wheel_separation>
  <wheelbase>0.22</wheelbase>
  <wheel_radius>0.07618</wheel_radius>
  <topic>/model/robot/cmd_vel</topic>
  <odom_topic>/model/robot/odometry</odom_topic>
  <frame_id>odom</frame_id>
  <child_frame_id>imu</child_frame_id>
  <odom_publish_frequency>50</odom_publish_frequency>
</plugin>
```

Add `ros_gz_bridge`, `geometry_msgs`, `nav_msgs`, `tf2_ros`, and `rclpy` as runtime dependencies.

- [x] **Step 4: Verify conversion and tests pass**

Run: `gz sdf -p src/mysystem/urdf/robot.urdf` and the test command from Step 2.

Expected: conversion prints an SDF plugin with the mecanum configuration and all structural tests pass.

### Task 2: Odom-To-TF Broadcaster

**Files:**
- Create: `src/mysystem/scripts/odom_to_tf.py`
- Modify: `src/mysystem/CMakeLists.txt`
- Test: `src/mysystem/test/test_odom_to_tf.py`

**Interfaces:**
- Consumes: `nav_msgs/msg/Odometry` from `/odom`.
- Produces: `geometry_msgs/msg/TransformStamped` through `tf2_ros.TransformBroadcaster`, with `header.frame_id == "odom"` and `child_frame_id == "imu"`.

- [x] **Step 1: Write a failing transform test**

```python
message = make_odometry(stamp_sec=12, x=1.2, y=-0.4, yaw_quaternion=(0, 0, 0.5, 0.866))
transform = odometry_to_transform(message)
assert transform.header.frame_id == "odom"
assert transform.child_frame_id == "imu"
assert transform.transform.translation.x == 1.2
assert transform.transform.rotation.z == 0.5
```

- [x] **Step 2: Run the unit test to verify it fails**

Run: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/mysystem/test/test_odom_to_tf.py -q`

Expected: import failure because `odom_to_tf.py` does not exist.

- [x] **Step 3: Implement the focused ROS 2 node**

Create `odometry_to_transform(message: Odometry) -> TransformStamped` and an
`OdomToTf` node that subscribes to `/odom`, calls the helper, and sends the
result with `TransformBroadcaster`. Copy stamp, position, and quaternion from
the odometry pose. Install it with:

```cmake
install(PROGRAMS scripts/odom_to_tf.py DESTINATION lib/${PROJECT_NAME})
```

- [x] **Step 4: Verify the unit test passes**

Run: `source /opt/ros/jazzy/setup.bash && python3 -m pytest src/mysystem/test/test_odom_to_tf.py -q`

Expected: one conversion test passes without a ROS graph.

### Task 3: Simulation Launch Bridge

**Files:**
- Modify: `src/mysystem/launch/sim_worlds_launch.py`
- Modify: `src/mysystem/test/test_sim_world_assets.py`

**Interfaces:**
- Consumes: Gazebo `/model/robot/cmd_vel`, `/model/robot/odometry`, and `/clock`.
- Produces: ROS `/cmd_vel`, `/odom`, `/clock`, and the installed `odom_to_tf` process.

- [x] **Step 1: Write a failing launch-description test**

```python
assert any(
    isinstance(action, Node)
    and action.node_package == "ros_gz_bridge"
    and action.node_executable == "parameter_bridge"
    for action in description.entities
)
assert any(
    isinstance(action, Node)
    and action.node_package == "mysystem"
    and action.node_executable == "odom_to_tf.py"
    for action in description.entities
)
```

- [x] **Step 2: Run the test to verify it fails**

Run: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 src/mysystem/test/test_sim_world_assets.py`

Expected: neither bridge node nor TF broadcaster is in the launch description.

- [x] **Step 3: Add bridge and broadcaster actions**

Add a `ros_gz_bridge/parameter_bridge` node with:

```python
arguments=[
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    '/model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
]
```

Remap the command and odometry topics to `/cmd_vel` and `/odom`, then add a
`mysystem/odom_to_tf.py` node. Register both with `ld.add_action(...)`.

- [x] **Step 4: Verify launch tests pass**

Run: `python3 src/mysystem/test/test_sim_world_assets.py` after sourcing ROS
and the workspace install setup.

Expected: the launch description contains simulator, spawner, bridge, and TF
broadcaster nodes.

### Task 4: Build And Headless Motion Verification

**Files:**
- Modify: none unless an integration failure identifies a configuration issue.

**Interfaces:**
- Verifies: ROS `/cmd_vel` changes `/odom` and makes `odom -> imu` queryable.

- [x] **Step 1: Build and run package tests**

Run: `colcon build --packages-select mysystem --event-handlers console_direct+`
then `colcon test --packages-select mysystem --ctest-args -R 'test_sim_world_assets|test_odom_to_tf' --event-handlers console_direct+`.

Expected: build succeeds and all selected tests pass.

- [x] **Step 2: Run headless simulation smoke test**

Start `gz sim -s -r` with `world.world`, set `GZ_SIM_RESOURCE_PATH` to the
installed package share parent, spawn the URDF, run the bridge and TF node,
then publish a positive `linear.x` ROS Twist to `/cmd_vel`.

- [x] **Step 3: Assert runtime outputs**

Observe one nonzero `/odom` pose or twist sample after the command and query
`odom -> imu` with `tf2_echo`. Fail the verification if Gazebo logs contain
`Unable to find file with URI`, `MecanumDrive`, or plugin-loading errors.

- [x] **Step 4: Record final evidence without committing**

Run `git status --short` only to show file state if Git still exists. Do not
stage, commit, push, tag, or create remotes.
