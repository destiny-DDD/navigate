# Gazebo Sim Mecanum Drive Design

## Goal

Run the existing `mysystem` robot in Gazebo Sim as a four-wheel mecanum base.
ROS 2 clients command the simulator through `/cmd_vel`, receive `/odom`, and
receive a dynamic `odom -> imu` transform. The robot remains defined by the
existing `src/mysystem/urdf/robot.urdf`; no second SDF robot description is
introduced.

## Architecture

`robot.urdf` will contain a Gazebo Sim `MecanumDrive` plugin configuration.
The plugin controls the existing wheel joints and publishes Gazebo Transport
odometry. `sim_worlds_launch.py` starts the Gazebo bridge and a small ROS 2
transform broadcaster after launching the world and spawning the robot.

The data flow is:

```text
ROS /cmd_vel -> Gazebo /model/robot/cmd_vel -> MecanumDrive -> wheel joints
Gazebo /model/robot/odometry -> ROS /odom -> odom_to_tf -> odom -> imu
Gazebo /clock -> ROS /clock
```

## Mecanum Plugin

The plugin uses the Gazebo Sim system filename
`gz-sim-mecanum-drive-system` and class `gz::sim::systems::MecanumDrive`.
The existing URDF wheel positions determine its configuration:

| Plugin role | URDF joint |
| --- | --- |
| Front left | `joint_wheel_1` |
| Front right | `joint_wheel_4` |
| Back left | `joint_wheel_2` |
| Back right | `joint_wheel_3` |

The configuration values are:

| Parameter | Value | Source |
| --- | --- | --- |
| Wheel radius | `0.07618 m` | Wheel STL diameter is about `0.15235 m` |
| Wheel separation | `0.321 m` | Left and right wheel joint Y positions |
| Wheelbase | `0.22 m` | Front and rear wheel joint X positions |
| Command topic | `/model/robot/cmd_vel` | Gazebo model-scoped control topic |
| Odometry topic | `/model/robot/odometry` | Gazebo model-scoped output topic |
| Frame ID | `odom` | ROS navigation convention |
| Child frame ID | `imu` | Root link of the current URDF |

## ROS Integration

`mysystem` gains runtime dependencies on `ros_gz_bridge`, `geometry_msgs`,
`nav_msgs`, `tf2_ros`, and `rclpy`.

The bridge maps `/clock` from Gazebo to ROS, maps the model-scoped velocity
topic to ROS `/cmd_vel`, and maps the model-scoped odometry topic to ROS
`/odom`. The TF broadcaster subscribes to `/odom` and publishes only the
dynamic `odom -> imu` transform using the odometry timestamp and pose.

The broadcaster emits clear startup and subscription errors through normal ROS
logging. It never synthesizes odometry or control commands.

## Files

- `src/mysystem/urdf/robot.urdf`: Gazebo Sim mecanum plugin.
- `src/mysystem/launch/sim_worlds_launch.py`: bridge and TF broadcaster launch
  actions, following the repository launch formatting convention.
- `src/mysystem/scripts/odom_to_tf.py`: odometry-to-transform ROS 2 node.
- `src/mysystem/CMakeLists.txt`: install the executable Python script.
- `src/mysystem/package.xml`: runtime dependencies and resource export.
- `src/mysystem/test/test_sim_world_assets.py`: structural regression tests.

## Verification

1. Build `mysystem` and run its regression tests.
2. Validate the URDF-to-SDF conversion and confirm the mecanum plugin is
   retained with the intended joints and geometry.
3. Run Gazebo server-only, spawn the robot, start the bridge and TF node,
   publish forward and lateral ROS `/cmd_vel` commands, and confirm `/odom`
   changes and `odom -> imu` is available.
4. Confirm Gazebo logs contain no unresolved mesh or plugin errors.
