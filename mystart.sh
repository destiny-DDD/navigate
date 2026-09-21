#!/usr/bin/env bash
#
# 一键启动：先拉起系统层，再拉起控制层，两个同时运行。
#   mysystem  : robot_state_publisher + joint_state_publisher + foxglove_bridge  （后台）
#   mycontrol : Point-LIO + 底盘控制节点                                           （前台）
#
# Ctrl+C 退出时会自动把后台那个也收掉，不留孤儿进程。
#
# 用法：
#   ./mystart.sh                # 默认等待 3 秒
#   WAIT_SEC=8 ./mystart.sh     # 第一个 launch 起得慢时放宽等待

cd "$(dirname "$(readlink -f "$0")")" || exit 1  # 切到脚本所在目录，保证从任何路径调用都能找到 install/。

# 环境准备：ROS 本体 + 本工作区，缺一个 ros2 launch 都会报找不到包。
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"  # 优先用当前已 source 的发行版，没设就默认 jazzy。
if [ ! -f "$ROS_SETUP" ]; then  # 如果上面拼出来的路径不存在（比如 ROS_DISTRO 指向了没装的版本）。
    ROS_SETUP="/opt/ros/jazzy/setup.bash"  # 那就退回写死的 jazzy 路径再试一次。
fi  # 结束第一次回退判断。
if [ ! -f "$ROS_SETUP" ]; then  # 回退之后仍然找不到，说明这台机器根本没装 ROS。
    echo "[mystart] 找不到 ROS 环境：$ROS_SETUP" >&2  # 把错误原因打到标准错误，方便重定向时也能看到。
    exit 1  # 直接退出，不继续往下跑。
fi  # 结束第二次检查。
# shellcheck disable=SC1090  # 告诉 shellcheck 别警告“无法跟随非字面量路径的 source”。
source "$ROS_SETUP"  # 载入 ROS 本体环境，提供 ros2 命令和系统自带的包。

if [ ! -f install/setup.bash ]; then  # 本工作区的环境文件不存在，通常意味着还没构建过。
    echo "[mystart] 找不到 install/setup.bash，先跑一次 ./mybuild.sh" >&2  # 提示用户先构建。
    exit 1  # 没构建就没法启动，直接退出。
fi  # 结束工作区检查。
# shellcheck disable=SC1091  # 告诉 shellcheck 别警告“source 的文件此刻不存在”（它是运行时才生成的）。
source install/setup.bash  # 载入本工作区环境，让 ros2 能找到 mysystem / mycontrol 这两个包。

WAIT_SEC="${WAIT_SEC:-3}"  # 第一个 launch 起来后等几秒再拉第二个，可用环境变量覆盖，默认 3 秒。

# 收到 Ctrl+C 或脚本正常结束时，把后台的 mysystem 一起关掉。
cleanup() {  # 定义清理函数，注册到 trap 上，脚本无论如何退出都会执行。
    echo  # 先打个空行，避免和前面节点的日志挤在一起。
    echo "[mystart] 正在关闭 mysystem ..."  # 告诉用户后台那个 launch 正在被收掉。
    kill -INT "$SYSTEM_PID" 2>/dev/null  # 给后台的 ros2 launch 发 Ctrl+C 同样的信号，让它自己优雅退出。
    wait "$SYSTEM_PID" 2>/dev/null  # 等它真正退干净再结束脚本，避免留下僵尸进程。
}  # 结束清理函数定义。
trap cleanup EXIT  # 注册：脚本退出时（包括被 Ctrl+C 打断）自动调用上面的清理函数。

echo "[mystart] 启动 mysystem（模型 + Foxglove）..."  # 打印进度，让用户知道现在在起哪个层。
ros2 launch mysystem mysystem_love_launch.py &  # 后台启动系统层，末尾的 & 是关键，不然后面就执行不下去了。
SYSTEM_PID=$!  # 记下后台进程的 PID，后面 cleanup 要靠它来关。

echo "[mystart] 等待 ${WAIT_SEC} 秒让其就绪 ..."  # 打印进度，说明接下来要等一会儿。
sleep "$WAIT_SEC"  # 等待，让 robot_state_publisher 先把 robot_description 发布出来。

echo "[mystart] 启动 mycontrol（Point-LIO + 底盘）..."  # 打印进度，进入第二个 launch。
ros2 launch mycontrol mycontrol_launch.py  # 前台启动控制层，这里的输出会直接显示在当前终端。

# 第二个 launch 退出后，上面注册的 trap 会自动收掉第一个。
