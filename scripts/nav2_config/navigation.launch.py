"""启动 Nav2 导航（定位 + 规划 + 控制 + RViz）— 需要已有地图。

用法:
    source /opt/ros/humble/setup.bash
    ros2 launch scripts/nav2_config/navigation.launch.py map:=/path/to/map.yaml

定位方式 (localization:=...)：
    amcl  （默认）标准 ROS 做法：AMCL 粒子滤波在已有地图上做激光定位，发布 map→odom。
    odom  直接把 map→odom 固定为单位变换（当里程计可信时用；仿真里里程计就是真值，
          地图又是从同一个起点建的，所以这个模式定位误差 = 0，比赛脚本化跑动推荐）。

需要先启动 Isaac Sim + ros2_leisaac_bridge.py。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.dirname(os.path.realpath(__file__))
    nav2_config = os.path.join(config_dir, "nav2.yaml")
    rviz_config = os.path.join(config_dir, "navigation.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    # RViz 默认跟随 DISPLAY：无显示（纯 headless 容器）时自动跳过，避免 launch 报错
    with_rviz = LaunchConfiguration("rviz", default="true" if os.environ.get("DISPLAY") else "false")
    map_yaml = LaunchConfiguration("map")
    localization = LaunchConfiguration("localization", default="amcl")
    is_amcl = IfCondition(PythonExpression(["'", localization, "' == 'amcl'"]))
    is_odom = IfCondition(PythonExpression(["'", localization, "' == 'odom'"]))

    # 定位方式 A：AMCL 粒子滤波（标准 ROS 流程，输出 map→odom）
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_config, {"use_sim_time": use_sim_time}],
        condition=is_amcl,
    )

    # 定位方式 B：map→odom 固定为单位变换（"里程计即真值"）
    static_map_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=is_odom,
    )

    # Map server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "yaml_filename": map_yaml},
        ],
    )

    # Nav2 lifecycle manager（跟着定位方式管理不同的节点集合）
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            },
        ],
        condition=is_amcl,
    )
    lifecycle_manager_odom_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            },
        ],
        condition=is_odom,
    )

    # 使用 duojin01 的 MPPI 控制器参数启动 controller + planner
    controller_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_config, {"use_sim_time": use_sim_time}],
    )

    planner_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_config, {"use_sim_time": use_sim_time}],
    )

    behavior_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_config, {"use_sim_time": use_sim_time}],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_config, {"use_sim_time": use_sim_time}],
    )

    lifecycle_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["controller_server", "planner_server", "behavior_server", "bt_navigator"],
            },
        ],
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
            DeclareLaunchArgument("map", description="Path to map YAML file"),
            DeclareLaunchArgument(
                "localization", default_value="amcl", choices=["amcl", "odom"],
                description="amcl=粒子滤波定位（标准）；odom=固定 map→odom（里程计即真值）",
            ),
            amcl_node,
            static_map_odom_node,
            map_server_node,
            lifecycle_manager_node,
            lifecycle_manager_odom_node,
            controller_node,
            planner_node,
            behavior_node,
            bt_navigator_node,
            lifecycle_nav,
            rviz_node,
        ]
    )
