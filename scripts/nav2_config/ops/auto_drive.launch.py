"""自动行驶 —— 不用键盘也能让机器人按预设轨迹走（建图扫场 / 导航验证 / 定点测试）。

等价于原来的 `reproduce/ros2_auto_drive.sh`（逻辑逐行一致，只是把 argv 提成了 ROS 参数）。
用 ``/odom`` 闭环，并用 ``/scan`` 检查行驶方向净空：净空 < ``clearance`` 就提前停下
（运动学底盘会穿墙，靠这层兜住）。

⚠️ 本入口**默认不起桥接**（``bridge:=false``）—— 它是辅助操作，插进已有 ROS2 图里用。

用法::

    ros2 launch scripts/nav2_config/ops/auto_drive.launch.py pattern:=explore
    ros2 launch scripts/nav2_config/ops/auto_drive.launch.py pattern:=map
    ros2 launch scripts/nav2_config/ops/auto_drive.launch.py dist:=-0.5      # 直行（负=后退）0.5 m
    ros2 launch scripts/nav2_config/ops/auto_drive.launch.py strafe:=0.4     # 左平移 0.4 m
    ros2 launch scripts/nav2_config/ops/auto_drive.launch.py turn:=90        # 逆时针转 90°
    ros2 launch scripts/nav2_config/ops/auto_drive.launch.py vx:=0.2 wz:=0.5 sec:=3

参数:
    pattern      none / explore / map   默认 none（配合下面的单步参数用）
                 explore = 贪心挑最开阔方向走 0.6 m × 8（推荐，用于自动建图）
                 map     = 原地转 360° ×4，每次之间后退 0.5 m
    dist         直行距离 m（负数=后退）
    strafe       左平移距离 m（负数=右）
    turn         原地转角 度（负数=顺时针）
    vx/vy/wz     原始速度；配合 sec 使用
    sec          定时指令持续秒数
    clearance    安全净空 m      默认 0.35
    speed        平移速度 m/s    默认 0.20
    spin         旋转角速度 rad/s 默认 0.60
    bridge       是否顺带起桥接  默认 false
    sim_host / sim_port          默认 127.0.0.1 / 5560
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from launch_utils import as_bool, paths, python_script  # noqa: E402

_CONFIG_DIR, _REPO_ROOT, _NODES_DIR, _MAPS_DIR = paths(__file__)

#: 直接透传给节点的参数名（节点里都有同名 declare_parameter）
_NODE_PARAM_NAMES = (
    "pattern", "dist", "strafe", "turn", "vx", "vy", "wz", "sec",
    "clearance", "speed", "spin",
)


def _create_actions(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    actions = []
    if as_bool(get("bridge")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(_CONFIG_DIR, "bridge.launch.py")),
                launch_arguments={"sim_host": get("sim_host"), "sim_port": get("sim_port")}.items(),
            )
        )
    actions.append(
        python_script(
            os.path.join(_NODES_DIR, "auto_drive_node.py"),
            {name: get(name) for name in _NODE_PARAM_NAMES},
            name="auto_drive",
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("pattern", default_value="none",
                                  description="none / explore（贪心探索）/ map（原地转+后退）"),
            DeclareLaunchArgument("dist", default_value="0.0", description="直行距离 m（负=后退）"),
            DeclareLaunchArgument("strafe", default_value="0.0", description="左平移 m（负=右）"),
            DeclareLaunchArgument("turn", default_value="0.0", description="原地转角 度（负=顺时针）"),
            DeclareLaunchArgument("vx", default_value="0.0"),
            DeclareLaunchArgument("vy", default_value="0.0"),
            DeclareLaunchArgument("wz", default_value="0.0"),
            DeclareLaunchArgument("sec", default_value="0.0", description="定时指令持续 s"),
            DeclareLaunchArgument("clearance", default_value="0.35", description="安全净空 m"),
            DeclareLaunchArgument("speed", default_value="0.20", description="平移速度 m/s"),
            DeclareLaunchArgument("spin", default_value="0.60", description="旋转角速度 rad/s"),
            DeclareLaunchArgument("bridge", default_value="false",
                                  description="是否顺带起桥接（默认 false，插进已有 ROS2 图用）"),
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("sim_port", default_value="5560"),
            OpaqueFunction(function=_create_actions),
        ]
    )
