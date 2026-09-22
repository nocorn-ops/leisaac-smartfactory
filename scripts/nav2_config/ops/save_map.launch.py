"""存图 —— 把 slam_toolbox 建好的地图存成 .pgm + .yaml（并打印地图统计）。

**照 duojin01 的做法**：调 slam_toolbox 自带的 ``/slam_toolbox/save_map`` 服务，
`map_name` 默认 ``auto`` → **自动递增**（0 → 1 → 2 …），不会覆盖历史地图。

⚠️ 本入口**默认不起桥接**（``bridge:=false``）—— 它是"插进已有 ROS2 图里用"的辅助操作，
在**建图进程还在跑**的时候执行（它需要 ``/slam_toolbox/save_map`` 服务）。

用法::

    # 建图（另一个终端 / 另一个 docker exec）还在跑的时候：
    ros2 launch scripts/nav2_config/ops/save_map.launch.py                 # 存成 0.yaml
    ros2 launch scripts/nav2_config/ops/save_map.launch.py map_name:=arena
    ros2 launch scripts/nav2_config/ops/save_map.launch.py output_dir:=/tmp/maps

参数:
    map_name      地图名；留空或 auto = 自动递增   默认 auto
    output_dir    输出目录                       默认 scripts/nav2_config/maps
    wait_timeout  等服务和等存完的超时 s          默认 30.0
    bridge        是否顺带起桥接                  默认 false
    sim_host / sim_port                          默认 127.0.0.1 / 5560
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
            os.path.join(_NODES_DIR, "save_map_node.py"),
            {"map_name": get("map_name"), "output_dir": get("output_dir"),
             "wait_timeout": get("wait_timeout")},
            name="save_map",
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_name", default_value="auto",
                                  description="地图名；auto = 自动递增（0 → 1 → 2 …）"),
            DeclareLaunchArgument("output_dir", default_value=_MAPS_DIR, description="输出目录"),
            DeclareLaunchArgument("wait_timeout", default_value="30.0",
                                  description="等 /slam_toolbox/save_map 和等存完的超时 s"),
            DeclareLaunchArgument("bridge", default_value="false",
                                  description="是否顺带起桥接（默认 false，插进已有 ROS2 图用）"),
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("sim_port", default_value="5560"),
            OpaqueFunction(function=_create_actions),
        ]
    )
