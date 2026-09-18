# Repository Instructions

## ROS 2 Launch Files

- Use `src/mysystem/launch/mysystem_love_launch.py` as the formatting reference for ROS 2 launch files.
- Add concise Chinese inline comments that explain imports, launch arguments, actions, and the returned `LaunchDescription`.
- Create an empty `LaunchDescription`, add actions explicitly with `ld.add_action(...)`, then return `ld`.
