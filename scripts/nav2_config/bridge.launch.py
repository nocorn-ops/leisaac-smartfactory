"""公共组件：启动 Isaac Sim ↔ ROS2 桥接客户端（= duojin01 里 `base.launch.py` 的那部分）。

**这是整个 nav2_config 下唯一知道"怎么起桥接"的地方。** 以前
`ros2_keyboard_container.sh` / `ros2_mapping.sh` / `ros2_navigation.sh` 各自复制了一份
"起桥接 + 等 20 秒看 Connected"，还要靠人记"别再起第二个桥接"；现在统一由本文件负责，
其它入口用 ``bridge:=true/false`` 决定要不要带上它。

参数:
    sim_host   桥接服务端地址（宿主机跑 ros2_chassis_teleop.sh）  默认 127.0.0.1
    sim_port   桥接服务端端口                                    默认 5560
    rate_hz    ROS2 侧转发频率                                   默认 30.0

单独用（只起桥接，不带任何算法）::

    ros2 launch scripts/nav2_config/bridge.launch.py
    ros2 launch scripts/nav2_config/bridge.launch.py sim_host:=192.168.1.10 sim_port:=5560
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = os.path.dirname(os.path.realpath(__file__))
    repo_root = os.path.dirname(os.path.dirname(config_dir))
    bridge_py = os.path.join(repo_root, "scripts", "ros2_leisaac_bridge.py")

    sim_host = LaunchConfiguration("sim_host")
    sim_port = LaunchConfiguration("sim_port")
    rate_hz = LaunchConfiguration("rate_hz")

    bridge_process = ExecuteProcess(
        cmd=[
            # ★ sys.executable = ros2 launch 自己的 python（= ROS 的 python，一定有 rclpy）。
            #   不要写 "python3"：赛队机器上 conda 常驻时它会解析到 conda 的 python，
            #   没有 rclpy，桥接直接 ImportError 起不来。
            sys.executable,
            bridge_py,
            "--sim_host", sim_host,
            "--sim_port", sim_port,
            "--rate_hz", rate_hz,
        ],
        name="leisaac_bridge",
        output="screen",
        emulate_tty=True,
        # 仿真侧 SimBridgeServer 只服务一个客户端；多起一个时桥接自己会在 6 秒后报错提示。
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_host", default_value="127.0.0.1",
                                  description="Isaac Sim 桥接服务端地址"),
            DeclareLaunchArgument("sim_port", default_value="5560",
                                  description="Isaac Sim 桥接服务端端口"),
            DeclareLaunchArgument("rate_hz", default_value="30.0",
                                  description="ROS2 侧转发频率 Hz"),
            bridge_process,
        ]
    )
