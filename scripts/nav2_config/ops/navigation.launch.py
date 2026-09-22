"""导航（Nav2）—— 一条命令搞定（桥接 + 地图自动解析 + 定位 + 规划 + 控制 + RViz ± 自动发目标）。

等价于原来的 `reproduce/ros2_navigation.sh`，但：
- ``map`` **留空就自动挑地图**（数字名最大优先，否则最近修改），不用再敲路径；
- ``goal`` 直接给目标点，不用再开一个终端跑 ros2_send_goal.sh；
- 参数命名与 teleop / mapping 统一。

用法::

    # ① 前台：起 Nav2 + RViz，然后在 RViz 里点 "2D Goal Pose"
    ros2 launch scripts/nav2_config/ops/navigation.launch.py rviz:=true

    # ② 无人值守：归位 → 起 Nav2 → 发目标点 → 打印结果
    ros2 launch scripts/nav2_config/ops/navigation.launch.py goal:="-0.7 0.0 0"
    ros2 launch scripts/nav2_config/ops/navigation.launch.py goal:="0.75 -1.6 0" localization:=odom

    # ③ 指定地图 / 插进已有 ROS2 图
    ros2 launch scripts/nav2_config/ops/navigation.launch.py map:=maps/kitchen.yaml
    ros2 launch scripts/nav2_config/ops/navigation.launch.py bridge:=false rviz:=true

参数:
    sim_host / sim_port  桥接地址/端口       默认 127.0.0.1 / 5560
    bridge               是否起桥接           默认 true
    map                  地图 yaml           默认空 = 自动挑（数字名最大，否则最近修改）
    localization         定位方式             默认 amcl（amcl=粒子滤波 / odom=固定 map→odom，零误差）
    rviz                 是否开 RViz          默认 false（要看图就加 rviz:=true）
    use_sim_time         默认 false（仿真侧不发 /clock）
    reset_first          起 Nav2 前先归位     默认 true（地图是从起点建的，定位按 (0,0,0) 起步）
    nav_delay_sec        reset_first 时等多久再起 Nav2   默认 3.0
    goal                 目标点 "x y [yaw_deg]"；留空则不发（自己在 RViz 点）
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from launch_utils import as_bool, paths, python_script  # noqa: E402
from map_paths import get_maps_dir, resolve_map_yaml  # noqa: E402

_CONFIG_DIR, _REPO_ROOT, _NODES_DIR, _MAPS_DIR = paths(__file__)


def _parse_goal(text: str) -> tuple[float, float, float] | None:
    """解析 "x y [yaw_deg]"；空字符串 = 不发目标点。"""
    raw = (text or "").strip()
    if not raw:
        return None
    parts = raw.replace(",", " ").split()
    if len(parts) < 2 or len(parts) > 3:
        raise RuntimeError(
            f'goal 参数格式应为 "x y [yaw_deg]"，收到 {raw!r}。'
            f'例如 goal:="-0.7 0.0 0"'
        )
    try:
        x, y = float(parts[0]), float(parts[1])
        yaw = float(parts[2]) if len(parts) == 3 else 0.0
    except ValueError as exc:
        raise RuntimeError(f"goal 参数里有非数字：{raw!r}") from exc
    return x, y, yaw


def _create_actions(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    sim_host = get("sim_host")
    sim_port = get("sim_port")
    use_bridge = as_bool(get("bridge"))
    localization = get("localization")
    rviz = get("rviz")
    use_sim_time = get("use_sim_time")
    reset_first = as_bool(get("reset_first"))
    nav_delay_sec = get("nav_delay_sec")
    goal = _parse_goal(get("goal"))

    # 地图：留空 = 自动挑最新（数字名最大优先），挑不到会抛出带指引的错误
    maps_dir = get_maps_dir(_CONFIG_DIR)
    if not os.path.isdir(maps_dir) or not os.listdir(maps_dir):
        raise RuntimeError(
            f"{maps_dir} 下没有任何地图 —— 先建图：\n"
            f"  ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true"
        )
    map_yaml = resolve_map_yaml(get("map"), _CONFIG_DIR)

    actions = []

    # 目录里可能有好几张图（数字图 + 命名图），必须让人看见**到底加载了哪张**
    actions.append(
        LogInfo(msg=f"[nav] 使用地图: {map_yaml}"
                    f"（localization={localization}，reset_first={reset_first}）")
    )

    if use_bridge:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(_CONFIG_DIR, "bridge.launch.py")),
                launch_arguments={"sim_host": sim_host, "sim_port": sim_port}.items(),
            )
        )

    if reset_first:
        actions.append(
            python_script(os.path.join(_NODES_DIR, "reset_sim_node.py"), None, name="reset_sim")
        )

    # 复用已经验证过的 Nav2 组件（scripts/nav2_config/navigation.launch.py）
    nav_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_CONFIG_DIR, "navigation.launch.py")),
        launch_arguments={
            "map": map_yaml,
            "localization": localization,
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    # 归位是瞬移（实测 0.2 s），等一小会再起 Nav2，保证定位从 (0,0,0) 起步
    actions.append(TimerAction(period=nav_delay_sec if reset_first else "0.0",
                               actions=[nav_include]))

    if goal is not None:
        actions.append(
            python_script(
                os.path.join(_NODES_DIR, "goal_client_node.py"),
                {"x": goal[0], "y": goal[1], "yaw_deg": goal[2]},
                name="goal_client",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("sim_port", default_value="5560"),
            DeclareLaunchArgument("bridge", default_value="true",
                                  description="是否启动桥接（已有桥接在跑时用 false）"),
            DeclareLaunchArgument("map", default_value="",
                                  description="地图 yaml；留空 = 自动挑（数字名最大，否则最近修改）"),
            DeclareLaunchArgument("localization", default_value="amcl", choices=["amcl", "odom"],
                                  description="amcl=粒子滤波定位（标准）；odom=固定 map→odom（零误差）"),
            DeclareLaunchArgument("rviz", default_value="false",
                                  description="是否开 RViz（默认关；容器里开 RViz 需 --gpus all + X11 授权）"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("reset_first", default_value="true",
                                  description="起 Nav2 前先让底盘归位（与建图同一起点）"),
            DeclareLaunchArgument("nav_delay_sec", default_value="3.0",
                                  description="reset_first 时等多久再起 Nav2"),
            DeclareLaunchArgument("goal", default_value="",
                                  description='目标点 "x y [yaw_deg]"（map 坐标系）；留空 = 不发'),
            OpaqueFunction(function=_create_actions),
        ]
    )
