```
ros2 pkg create --build-type ament_cmake --dependencies rclcpp --license Apache-2.0 

git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2

git clone https://github.com/manifoldsdk/odin_ros_driver.git catkin_ws/src/odin_ros_driver
```

9.21 吃shi日记 odin1显示usb找不到，lsusb发现确实没有，但是线都接了，配置文件也写了 cnm线有问题，换了根线立马好了。
现在打开rviz，没有odin数据（ssh远程连接的nuc），首先还留有之前rmw的环境变量，导致节点通信不走DDS，现在换成DDS，能找到节点了，但是没有值，实际上好像有值，但是只有点云图，RGB貌似报错了，正在排查问题，还有就是odin数据太多，nuc处理不过来，终端一直报队列满了的警告，警告问题是网络原因，把ssh换成便携屏警告就消失了。360也是同样问题，换成便携屏立马有点云了。
9.22 和电控通信，使用dmesg,usbtop,minicom查数据是对的，多吃吃底层是对的，通信通了，有几个远程调试的手段，Xterminal+foxglove todesk Remminna
9.23 你妈，昨天写了和下位机通信的代码，今天一看没法通信了，查了一下发现代码没了，怀疑昨天只传了主仓库，没传子仓库，今天拉代码又把之前代码覆盖了，导致我又得重新写通信。ai改了nav2_params，把恢复行为的spin删了，导致bt异常，导航没有启动成功。 最新一轮运行中，behavior_server 只加载了 wait，但默认导航行为树还需要 spin。bt_navigator 因找不到 spin action server 而激活失败，生命周期管理器随即中止启动：导航日志 (component_container_isolated_17421_1790167192540.log:219)、报错位置 (component_container_isolated_17421_1790167192540.log:289)。你在约 20:41 点的目标已由 RViz 发出，但导航服务处于 inactive 状态，目标被拒绝，根本没有进入全局规划：目标记录 (rviz2_8496_1790165704191.log:63)、拒绝记录
(component_container_isolated_17421_1790167192540.log:239)。前一轮运行也反复拒绝目标，说明不是偶发的规划失败。先检查 Nav2 参数中 behavior_server.behavior_plugins 与 bt_navigator 使用的行为树是否一致；当前行为树需要启用 spin。AMCL 的效果差有明确的时间同步/TF 异常迹象。 多次手动设置初始位姿时，AMCL 报 Failed to transform initial pose in time，所需时刻比 camera_init → base_link 可用的最新 TF 晚约 30–50 ms：AMCL 警告
(component_container_isolated_8761_1790165710854.log:117)。全局和局部代价地图也持续因 TF 缓存时间不匹配丢弃 base_link 数据：丢帧记录 (component_container_isolated_8761_1790165710854.log:319)。这会影响定位输入及障碍物更新，值得优先排查 odom/TF、/scan
的时间戳和发布延迟。

9.23 历史上最沉重的一天，nuc挂了111


ros2 topic pub --once /move_mode std_msgs/msg/Int32 "{data: 1}"