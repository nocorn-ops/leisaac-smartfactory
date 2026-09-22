#!/usr/bin/env bash
# 说明：本脚本是**发单个目标点的辅助工具**（没有等价的 ops launch）。
#   无人值守一条命令走完"归位→起 Nav2→发目标点→报结果"用：
#     ros2 launch scripts/nav2_config/ops/navigation.launch.py goal:="X Y YAW"
# 【容器侧 · 发目标点】在 map 坐标系里给 Nav2 一个目标点，等它走完并打印结果。
#
# 用法:
#   bash /work/reproduce/ros2_send_goal.sh X Y [YAW角度]     # 例如 bash ros2_send_goal.sh -0.8 0.0 0
#   bash /work/reproduce/ros2_send_goal.sh --check           # 只看导航状态（不发目标点）
#
# 说明: 建图与导航都从同一个物理起点开始 → 机器人在 map 原点、朝向 map +x；
#       所以"往机器人身后走 0.8 m" 就是 x=-0.8。
# 退出码: 0 = 到达, 1 = 失败/被中止, 2 = 参数错误
set -uo pipefail

if [ "${1:-}" = "--check" ]; then
    echo "[goal] ROS2 节点:"
    timeout 5 ros2 node list 2>/dev/null | sed 's/^/       /'
    echo "[goal] lifecycle 状态:"
    for n in map_server amcl controller_server planner_server bt_navigator; do
        printf "       %-18s %s\n" "$n" "$(timeout 3 ros2 lifecycle get /$n 2>/dev/null | head -1 || echo '未运行')"
    done
    echo "[goal] 当前 map→base_footprint（定位结果；amcl 模式=粒子滤波，odom 模式=固定 map→odom）:"
    timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null \
        | grep -m1 -A1 "Translation" | tr '\n' ' ' | sed 's/^/       /'
    echo
    exit 0
fi

if [ $# -lt 2 ]; then echo "用法: $0 X Y [YAW角度]   |   $0 --check" >&2; exit 2; fi

X="$1"; Y="$2"; YAW_DEG="${3:-0}"
echo "[goal] 目标: map 坐标系 ($X, $Y) yaw=${YAW_DEG}°"

python3 - "$X" "$Y" "$YAW_DEG" <<'PY'
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

x, y, yaw_deg = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
yaw = math.radians(yaw_deg)

rclpy.init()
node = Node("leisaac_send_goal")
client = ActionClient(node, NavigateToPose, "/navigate_to_pose")

node.get_logger().info("等待 /navigate_to_pose 动作服务器 ...")
if not client.wait_for_server(timeout_sec=30.0):
    node.get_logger().error("❌ Nav2 没起来（/navigate_to_pose 不存在）—— 先跑 ros2_navigation.sh")
    rclpy.shutdown(); sys.exit(1)

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.position.x = x
goal.pose.pose.position.y = y
goal.pose.pose.position.z = 0.0
goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

t0 = time.time()
last = [0.0]

def on_feedback(msg):
    fb = msg.feedback
    now = time.time()
    if now - last[0] > 1.0:
        last[0] = now
        node.get_logger().info(
            f"  前进中 {now - t0:5.1f}s  剩余 {fb.distance_remaining:5.2f} m  "
            f"恢复行为 {fb.number_of_recoveries} 次"
        )

node.get_logger().info("已发送目标点，等待结果 ...")
send = client.send_goal_async(goal, feedback_callback=on_feedback)
rclpy.spin_until_future_complete(node, send, timeout_sec=20.0)
handle = send.result()
if handle is None or not handle.accepted:
    node.get_logger().error("❌ 目标点被拒绝（bt_navigator 处于 active 吗？）")
    rclpy.shutdown(); sys.exit(1)

res = handle.get_result_async()
rclpy.spin_until_future_complete(node, res)
status = res.result().status
names = {2: "UNKNOWN", 3: "ACCEPTED", 4: "SUCCEEDED ✅", 5: "CANCELED", 6: "ABORTED ❌"}
node.get_logger().info(f"结果: {names.get(status, status)}  用时 {time.time() - t0:.1f}s")
rclpy.shutdown()
sys.exit(0 if status == 4 else 1)
PY
RC=$?
echo "[goal] 退出码 $RC（0 = 已到达）"
exit $RC
