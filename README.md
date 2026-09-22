```
ros2 pkg create --build-type ament_cmake --dependencies rclcpp --license Apache-2.0 

git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2

git clone https://github.com/manifoldsdk/odin_ros_driver.git catkin_ws/src/odin_ros_driver
```

9.21 吃shi日记 odin1显示usb找不到，lsusb发现确实没有，但是线都接了，配置文件也写了 cnm线有问题，换了根线立马好了。
现在打开rviz，没有odin数据（ssh远程连接的nuc），首先还留有之前rmw的环境变量，导致节点通信不走DDS，现在换成DDS，能找到节点了，但是没有值，实际上好像有值，但是只有点云图，RGB貌似报错了，正在排查问题，还有就是odin数据太多，nuc处理不过来，终端一直报队列满了的警告，警告问题是网络原因，把ssh换成便携屏警告就消失了。360也是同样问题，换成便携屏立马有点云了。
9.22 和电控通信，使用dmesg,usbtop,minicom查数据是对的，多吃吃底层是对的

ros2 topic pub --once /move_mode std_msgs/msg/Int32 "{data: 1}"