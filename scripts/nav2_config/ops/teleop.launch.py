"""键盘遥控底盘 —— 一条命令搞定（桥接 + 键盘）。

等价于原来的 `reproduce/ros2_keyboard_container.sh`，但不用再单独记"哪个脚本起桥接"：
本入口默认自己起桥接（``bridge:=true``），已经在别处起了就 ``bridge:=false``。

用法::

    # ⚠️ 容器必须用 -it 起（键盘要真 TTY）：
    docker run -it --rm --network host -v <仓库>:/work -w /work ros2-humble-dev:latest \
      bash -lc "source /opt/ros/humble/setup.bash && \\
                ros2 launch scripts/nav2_config/ops/teleop.launch.py"

    ros2 launch scripts/nav2_config/ops/teleop.launch.py speed:=0.3 turn:=0.5
    ros2 launch scripts/nav2_config/ops/teleop.launch.py bridge:=false   # 插进已有 ROS2 图

按键（teleop_twist_keyboard）::

    i 前进   , 后退   j 左转   l 右转   u/o/m/. 斜向
    k 停     q/z 加减速度     w/x 只调线速度     e/c 只调角速度     Ctrl+C 退出

参数:
    sim_host  桥接地址   默认 127.0.0.1
    sim_port  桥接端口   默认 5560
    bridge    是否起桥接  默认 true
    speed     线速度 m/s 默认 0.5
    turn      角速度 rad/s 默认 1.0

★ 为什么键盘用 ExecuteProcess + ``< /dev/tty`` 而不是 ``Node(...)``：
  ROS2 Humble 的 launch 用 asyncio 的 subprocess 起子进程，**stdin 默认是一个新建的 pipe**
  （`osrf_pycommon/.../async_execute_process_asyncio/impl.py` 里
  `_async_execute_process_pty = _async_execute_process_nopty`，而 nopty 不传 `stdin`，
  于是吃 asyncio 的默认 `stdin=PIPE`）。所以无论 ``emulate_tty`` 设 True/False，
  launch 起的子进程 ``sys.stdin.isatty()`` 都是 **False** —— 而 teleop_twist_keyboard 第 121 行
  会 ``termios.tcgetattr(sys.stdin)``，直接 ``termios.error: (25, 'Inappropriate ioctl for device')`` 挂掉。
  （实测：`Node(...)` 与 `ExecuteProcess(emulate_tty=True)` 两种都挂。）
  绕法就是让子进程自己重开控制终端：``< /dev/tty`` → ``isatty() = True``（已实测）。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from launch_utils import KEYBOARD_HELP, as_bool, keyboard_process  # noqa: E402

_CONFIG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def _create_actions(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    sim_host = get("sim_host")
    sim_port = get("sim_port")
    use_bridge = as_bool(get("bridge"))

    actions = []
    if use_bridge:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(_CONFIG_DIR, "bridge.launch.py")),
                launch_arguments={"sim_host": sim_host, "sim_port": sim_port}.items(),
            )
        )

    # 键盘进程用 < /dev/tty 重开控制终端（原因见 launch_utils.keyboard_process 的注释）
    actions.append(LogInfo(msg=KEYBOARD_HELP))
    actions.append(keyboard_process(get("speed"), get("turn")))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("sim_port", default_value="5560"),
            DeclareLaunchArgument("bridge", default_value="true",
                                  description="是否启动桥接（已有桥接在跑时用 false）"),
            DeclareLaunchArgument("speed", default_value="0.5", description="线速度 m/s"),
            DeclareLaunchArgument("turn", default_value="1.0", description="角速度 rad/s"),
            OpaqueFunction(function=_create_actions),
        ]
    )
