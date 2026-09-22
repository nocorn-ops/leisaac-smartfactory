#!/usr/bin/env bash
# ⚠️ 旧入口（保留可用）。推荐等价的 `ros2 launch scripts/nav2_config/ops/reset_sim.launch.py`。
#    对照见 ops/README.md §5。
# 【容器侧 · 归位】让仿真里的底盘瞬移回**初始位姿**（不用重启 Isaac Sim）。
#
# 用途：建图走完一圈后，先存图，再把机器人送回起点，然后就能直接起导航
#       （Nav2 的 AMCL 初始位姿是 (0,0,0)，只有机器人真的在起点才成立）。
#
# 用法:
#   bash /work/reproduce/ros2_reset_sim.sh
#
# 需要宿主机正在跑 ros2_chassis_teleop.sh（桥接 5560）并且本容器里已有桥接客户端
# （ros2_mapping.sh / ros2_navigation.sh 会自己起）。
set -uo pipefail

command -v ros2 >/dev/null 2>&1 || { echo "[ERROR] 先 source /opt/ros/humble/setup.bash" >&2; exit 1; }

python3 - <<'PY'
import math, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty

rclpy.init()
node = Node("leisaac_reset_sim")
pub = node.create_publisher(Empty, "/leisaac/reset", 10)
state = {"odom": None}
node.create_subscription(Odometry, "/odom", lambda m: state.__setitem__("odom", m), 10)

# 先空转 1.5 s，确保订阅/publisher 都已建立连接
t0 = time.time()
while time.time() - t0 < 1.5:
    rclpy.spin_once(node, timeout_sec=0.1)

if state["odom"] is None:
    node.get_logger().error("❌ 收不到 /odom —— 桥接没连上 Isaac Sim？")
    rclpy.shutdown(); sys.exit(1)

pub.publish(Empty())
node.get_logger().info("已发送复位请求 → 等底盘回到初始位姿 ...")

ok = False
t0 = time.time()
while time.time() - t0 < 12.0:
    rclpy.spin_once(node, timeout_sec=0.1)
    m = state["odom"]
    if m is None:
        continue
    x, y = m.pose.pose.position.x, m.pose.pose.position.y
    q = m.pose.pose.orientation
    yaw = math.degrees(math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z))
    if abs(x) < 0.05 and abs(y) < 0.05 and abs(yaw) < 5.0:
        node.get_logger().info(f"✅ 已归位：odom=({x:+.3f}, {y:+.3f}) yaw={yaw:+.1f}°（用时 {time.time() - t0:.1f}s）")
        ok = True
        break
    time.sleep(0.2)

if not ok:
    m = state["odom"]
    x, y = m.pose.pose.position.x, m.pose.pose.position.y
    q = m.pose.pose.orientation
    yaw = math.degrees(math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z))
    node.get_logger().warn(f"⚠️ 12 秒内没回到起点，当前 odom=({x:+.3f}, {y:+.3f}) yaw={yaw:+.1f}°")

rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
