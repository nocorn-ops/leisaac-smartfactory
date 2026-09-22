"""底盘归位 —— 让仿真里的底盘瞬移回**初始位姿**（不用重启 Isaac Sim）。

等价于原来的 `reproduce/ros2_reset_sim.sh`。用途：建图走完一圈后，先存图，再把机器人送回起点，
然后就能直接起导航（Nav2 的定位假设机器人在 map 原点、朝向 map +x）。

⚠️ 本入口**默认不起桥接**（``bridge:=false``）—— 它是辅助操作，插进已有 ROS2 图里用。

用法::

    ros2 launch scripts/nav2_config/ops/reset_sim.launch.py
    ros2 launch scripts/nav2_config/ops/reset_sim.launch.py bridge:=true   # 顺带起桥接

参数:
    timeout_sec   等归位超时 s    默认 12.0
    xy_tol        位置容差 m      默认 0.05
    yaw_tol_deg   朝向容差 度     默认 5.0
    bridge        是否顺带起桥接  默认 false
    sim_host / sim_port           默认 127.0.0.1 / 5560
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
            os.path.join(_NODES_DIR, "reset_sim_node.py"),
            {"timeout_sec": get("timeout_sec"), "xy_tol": get("xy_tol"),
             "yaw_tol_deg": get("yaw_tol_deg")},
            name="reset_sim",
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("timeout_sec", default_value="12.0", description="等归位超时 s"),
            DeclareLaunchArgument("xy_tol", default_value="0.05", description="位置容差 m"),
            DeclareLaunchArgument("yaw_tol_deg", default_value="5.0", description="朝向容差 度"),
            DeclareLaunchArgument("bridge", default_value="false",
                                  description="是否顺带起桥接（默认 false，插进已有 ROS2 图用）"),
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("sim_port", default_value="5560"),
            OpaqueFunction(function=_create_actions),
        ]
    )
