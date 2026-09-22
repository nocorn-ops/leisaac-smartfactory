"""启动 SLAM 建图 + RViz 可视化 — 一键建图。

用法:
    source /opt/ros/humble/setup.bash
    ros2 launch scripts/nav2_config/mapping.launch.py              # 有 DISPLAY 就自动开 RViz
    ros2 launch scripts/nav2_config/mapping.launch.py rviz:=false  # 纯 headless

需要先启动 Isaac Sim + ros2_leisaac_bridge.py。容器里开 RViz 记得加 --gpus all。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.dirname(os.path.realpath(__file__))
    slam_config = os.path.join(config_dir, "slam_toolbox.yaml")
    rviz_config = os.path.join(config_dir, "mapping.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    # RViz 默认跟随 DISPLAY：无显示（纯 headless 容器）时自动跳过，避免 launch 报错
    with_rviz = LaunchConfiguration("rviz", default="true" if os.environ.get("DISPLAY") else "false")
    slam_params = LaunchConfiguration("slam_params_file", default=slam_config)

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        condition=IfCondition(with_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true" if os.environ.get("DISPLAY") else "false",
                                  description="是否启动 RViz（无 X11 显示时自动关闭）"),
            DeclareLaunchArgument("slam_params_file", default_value=slam_config,
                                  description="slam_toolbox 参数文件"),
            slam_node,
            rviz_node,
        ]
    )
