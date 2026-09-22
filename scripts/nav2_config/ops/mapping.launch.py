"""建图（SLAM）—— 一条命令搞定（桥接 + slam_toolbox + RViz + 键盘 / 或自动探索 → 存图 → 归位）。

等价于原来的 `reproduce/ros2_mapping.sh`。**照 duojin01 的思路**，这个入口自己把整条链组合好：
  · 桥接只有一处（`bridge.launch.py`），别的入口 include 它，不用记"谁起桥接"；
  · **手动建图时入口自己带遥控**（对应 duojin01 `sim_mapping.launch.py` 固定 `use_teleop:=true`），
    一条命令 = 桥接 + SLAM + RViz + 键盘，不用再开第二个终端、也不用记 `bridge:=false`；
  · **存图走 slam_toolbox 自带服务**（对应 duojin01 `save_map_client_node`），`map_name:=auto` 自动递增。

用法::

    # ① 手动建图（一条命令，含键盘）：开着车走一圈，走完用 ops/save_map.launch.py 存图
    ros2 launch scripts/nav2_config/ops/mapping.launch.py rviz:=true

    # ② 无人值守：自动探索扫场 → 存图 → 归位 → 自动退出（等价 ros2_mapping.sh --auto）
    ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true
    ros2 launch scripts/nav2_config/ops/mapping.launch.py auto:=true pattern:=map

    # ③ 不要键盘（例如遥控在别处 / 无 -it 的容器）／ 已经有一个桥接在跑
    ros2 launch scripts/nav2_config/ops/mapping.launch.py teleop:=false rviz:=true
    ros2 launch scripts/nav2_config/ops/mapping.launch.py bridge:=false rviz:=true

参数:
    sim_host / sim_port  桥接地址/端口        默认 127.0.0.1 / 5560
    bridge               是否起桥接            默认 true
    rviz                 是否开 RViz           默认 false（要看图就加 rviz:=true）
    use_sim_time         默认 false（仿真侧不发 /clock）
    teleop               手动建图是否带键盘     默认 **true**（auto:=true 时自动忽略）
    teleop_speed / teleop_turn  键盘速度       默认 0.5 m/s / 1.0 rad/s
    auto                 是否无人值守自动探索   默认 false
    pattern              auto 时的行驶方式      默认 explore（explore=贪心挑最开阔方向 / map=原地转+后退）
    drive_speed          自动行驶速度 m/s      默认 0.2
    drive_clearance      自动行驶安全净空 m     默认 0.35
    drive_spin           自动行驶转速 rad/s    默认 0.6
    save                 auto 走完是否自动存图   默认 true
    reset                auto 存完是否自动归位   默认 true
    exit_when_done       auto 全流程走完自动退出  默认 true（无人值守要的就是这个）
    map_name             存图名字              默认 auto（自动递增 0 → 1 → 2 …）
    output_dir           存图目录              默认 scripts/nav2_config/maps
    wait_timeout         存图等服务的超时 s     默认 30.0

⚠️ `teleop:=true` 要求容器用 `docker run -it` 起（键盘要真 TTY，见 `launch_utils.keyboard_process`）。
⚠️ `auto:=false`（默认）时 `save` / `reset` / `map_name` 不起作用 —— 手动建图的存图与归位请分别用
   `ops/save_map.launch.py` 和 `ops/reset_sim.launch.py`（对应 duojin01 把 save_map 单独做成一个 launch）。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from launch_utils import (  # noqa: E402
    KEYBOARD_HELP,
    as_bool,
    keyboard_process,
    paths,
    python_script,
)

_CONFIG_DIR, _REPO_ROOT, _NODES_DIR, _MAPS_DIR = paths(__file__)


def _create_actions(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731

    sim_host = get("sim_host")
    sim_port = get("sim_port")
    use_bridge = as_bool(get("bridge"))
    rviz = get("rviz")
    use_sim_time = get("use_sim_time")
    auto = as_bool(get("auto"))
    pattern = get("pattern")
    drive_speed = get("drive_speed")
    drive_clearance = get("drive_clearance")
    drive_spin = get("drive_spin")
    save = as_bool(get("save"))
    reset = as_bool(get("reset"))
    exit_when_done = as_bool(get("exit_when_done"))
    map_name = get("map_name")
    output_dir = get("output_dir")
    wait_timeout = get("wait_timeout")
    use_teleop = as_bool(get("teleop"))

    # ★ duojin01 的思路：**建图入口自己带遥控**（他们的 sim_mapping.launch.py 固定 use_teleop:=true），
    #   不让人去记"另开一个终端跑键盘、而且必须 bridge:=false"。
    #   auto 与 teleop 互斥（两个都在发 /cmd_vel），auto 优先。
    if auto and use_teleop:
        use_teleop = False

    actions = []

    if use_bridge:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(_CONFIG_DIR, "bridge.launch.py")),
                launch_arguments={"sim_host": sim_host, "sim_port": sim_port}.items(),
            )
        )

    # 复用已经验证过的 slam + RViz 组件（scripts/nav2_config/mapping.launch.py）
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(_CONFIG_DIR, "mapping.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time, "rviz": rviz}.items(),
        )
    )

    # 手动建图：把键盘一起拉起来（一条命令 = 桥接 + SLAM + RViz + 键盘）
    if use_teleop:
        actions.append(
            LogInfo(msg="手动建图：开着车把场地走一圈，走完用 ops/save_map.launch.py 存图")
        )
        actions.append(LogInfo(msg=KEYBOARD_HELP))
        actions.append(keyboard_process(get("teleop_speed"), get("teleop_turn")))

    if auto:
        drive = python_script(
            os.path.join(_NODES_DIR, "auto_drive_node.py"),
            {"pattern": pattern, "speed": drive_speed,
             "clearance": drive_clearance, "spin": drive_spin},
            name="auto_drive",
        )
        actions.append(drive)

        # 扫场结束后按 **串行链** 走：auto_drive → 存图 → 归位 →（可选）退出。
        # ⚠️ 必须串行：原来把 save/reset 一起塞进 auto_drive 的 on_exit 里，两个进程是**并行**
        #    起来的（实测 pid 60/62 同时启动）—— 存图还没落盘就归位，slam_toolbox 会收到
        #    一次位姿跳变，可能把地图带歪。旧 shell 的 `save_map.sh` 然后 `reset_sim.sh` 是串行的。
        previous = drive
        if save:
            save_action = python_script(
                os.path.join(_NODES_DIR, "save_map_node.py"),
                {"map_name": map_name, "output_dir": output_dir,
                 "wait_timeout": wait_timeout},
                name="save_map",
            )
            actions.append(
                RegisterEventHandler(OnProcessExit(target_action=previous, on_exit=[save_action]))
            )
            previous = save_action
        if reset:
            reset_action = python_script(
                os.path.join(_NODES_DIR, "reset_sim_node.py"), None, name="reset_sim"
            )
            actions.append(
                RegisterEventHandler(OnProcessExit(target_action=previous, on_exit=[reset_action]))
            )
            previous = reset_action
        if exit_when_done:
            # 跑完自己退出（无人值守要的就是这个）：不然 bridge + slam 会一直挂着要人 Ctrl+C
            actions.append(
                RegisterEventHandler(
                    OnProcessExit(target_action=previous,
                                  on_exit=[Shutdown(reason="建图流程结束（存图 + 归位已完成）")])
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
            DeclareLaunchArgument("rviz", default_value="false",
                                  description="是否开 RViz（默认关；容器里开 RViz 需 --gpus all + X11 授权）"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("auto", default_value="false",
                                  description="无人值守：自动探索扫场 → 存图 → 归位"),
            DeclareLaunchArgument("teleop", default_value="true",
                                  description="手动建图时是否把键盘一起拉起来（auto:=true 时忽略）"),
            DeclareLaunchArgument("teleop_speed", default_value="0.5", description="键盘线速度 m/s"),
            DeclareLaunchArgument("teleop_turn", default_value="1.0", description="键盘角速度 rad/s"),
            DeclareLaunchArgument("pattern", default_value="explore",
                                  description="auto 时的行驶方式（explore / map）"),
            DeclareLaunchArgument("drive_speed", default_value="0.2", description="自动行驶速度 m/s"),
            DeclareLaunchArgument("drive_clearance", default_value="0.35", description="自动行驶安全净空 m"),
            DeclareLaunchArgument("drive_spin", default_value="0.6", description="自动行驶转速 rad/s"),
            DeclareLaunchArgument("save", default_value="true", description="auto 走完是否自动存图"),
            DeclareLaunchArgument("reset", default_value="true", description="auto 存完是否自动归位"),
            DeclareLaunchArgument("exit_when_done", default_value="true",
                                  description="auto 流程走完是否自动退出整个 launch"),
            DeclareLaunchArgument("map_name", default_value="auto",
                                  description="存图名字；auto = 自动递增（0 → 1 → 2 …）"),
            DeclareLaunchArgument("output_dir", default_value=_MAPS_DIR, description="存图目录"),
            DeclareLaunchArgument("wait_timeout", default_value="30.0",
                                  description="存图时等 /slam_toolbox/save_map 的超时 s"),
            OpaqueFunction(function=_create_actions),
        ]
    )
