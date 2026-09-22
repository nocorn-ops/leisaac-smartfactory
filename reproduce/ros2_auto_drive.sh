#!/usr/bin/env bash
# ⚠️ 旧入口（保留可用）。推荐等价的 `ros2 launch scripts/nav2_config/ops/auto_drive.launch.py`
#    （`pattern:=explore` 等参数同名）。对照见 ops/README.md §5。
# 【容器侧 · 自动行驶】不用键盘也能让机器人按预设轨迹走 —— 用于建图扫场 / 导航验证。
#
# 用 /odom 闭环（走够多少米/转够多少度就停），并用 /scan 检查行驶方向上的净空：
# 净空 < --clearance 就提前停下（运动学底盘会穿墙，靠这层兜住）。
#
# 用法:
#   bash /work/reproduce/ros2_auto_drive.sh --pattern explore    # 自动探索建图（贪心挑最开阔方向，推荐）
#   bash /work/reproduce/ros2_auto_drive.sh --pattern map        # 固定扫场（原地转 360° × 4 + 后退 0.5m × 3）
#   bash /work/reproduce/ros2_auto_drive.sh --dist -0.5          # 直行（负数=后退）0.5 m
#   bash /work/reproduce/ros2_auto_drive.sh --strafe 0.4         # 左平移 0.4 m（负数=右）
#   bash /work/reproduce/ros2_auto_drive.sh --turn 90            # 逆时针转 90°（负数=顺时针）
#   bash /work/reproduce/ros2_auto_drive.sh --vx 0.2 --wz 0.5 --sec 3   # 原始定时指令
# 可选: --clearance 0.35（安全净空，默认 0.35 m）  --speed 0.2（平移速度）  --spin 0.6（转速 rad/s）
set -uo pipefail

PATTERN=""; DIST=""; STRAFE=""; TURN=""
VX=0.0; VY=0.0; WZ=0.0; SEC=""
CLEARANCE=0.35; SPEED=0.20; SPIN=0.60

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pattern)   PATTERN="$2"; shift 2 ;;
        --dist)      DIST="$2"; shift 2 ;;
        --strafe)    STRAFE="$2"; shift 2 ;;
        --turn)      TURN="$2"; shift 2 ;;
        --vx)        VX="$2"; shift 2 ;;
        --vy)        VY="$2"; shift 2 ;;
        --wz)        WZ="$2"; shift 2 ;;
        --sec)       SEC="$2"; shift 2 ;;
        --clearance) CLEARANCE="$2"; shift 2 ;;
        --speed)     SPEED="$2"; shift 2 ;;
        --spin)      SPIN="$2"; shift 2 ;;
        -h|--help)   sed -n '2,15p' "$0"; exit 0 ;;
        *)           echo "[drive] 未知参数 $1" >&2; exit 2 ;;
    esac
done

command -v ros2 >/dev/null 2>&1 || { echo "[ERROR] 先 source /opt/ros/humble/setup.bash" >&2; exit 1; }

python3 - "$PATTERN" "$DIST" "$STRAFE" "$TURN" "$VX" "$VY" "$WZ" "$SEC" "$CLEARANCE" "$SPEED" "$SPIN" <<'PY'
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

pattern, dist, strafe, turn, vx, vy, wz, sec, clearance, speed, spin = sys.argv[1:12]
dist = float(dist) if dist else None
strafe = float(strafe) if strafe else None
turn = float(turn) if turn else None
vx, vy, wz = float(vx), float(vy), float(wz)
sec = float(sec) if sec else None
clearance, speed, spin = float(clearance), float(speed), float(spin)


class AutoDrive(Node):
    def __init__(self):
        super().__init__("leisaac_auto_drive")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.x = self.y = self.yaw = None
        self.scan = None
        self._yaw_raw = None
        self._yaw_cont = 0.0
        self.finished = False

    def _on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        y = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        if self._yaw_raw is not None:
            d = y - self._yaw_raw
            d = (d + math.pi) % (2 * math.pi) - math.pi
            self._yaw_cont += d
        self._yaw_raw = y
        self.yaw = self._yaw_cont

    def _on_scan(self, m):
        self.scan = m

    def sector_min(self, psi_deg, half=40.0):
        """扫描数据里以 psi_deg（本体坐标系，0=前，+=左）为中心 ±half 度扇区的最近距离。"""
        if self.scan is None or not self.scan.ranges:
            return float("inf")
        inc = self.scan.angle_increment
        best = float("inf")
        for i, r in enumerate(self.scan.ranges):
            if not math.isfinite(r):
                continue
            a = math.degrees(self.scan.angle_min + i * inc)
            d = (a - psi_deg + 180.0) % 360.0 - 180.0
            if abs(d) <= half:
                best = min(best, r)
        return best

    def send(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)


def wait_odom(node, timeout=15.0):
    t0 = time.time()
    while node.x is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.x is not None


def best_heading(node, half=25.0, n=72):
    """视野最开阔的行驶方向（本体坐标系角度, 该方向净空）。"""
    best = (0.0, -1.0)
    for k in range(n):
        psi = -180.0 + 360.0 * k / n
        c = node.sector_min(psi, half)
        if c > best[1]:
            best = (psi, c)
    return best


def explore(node, segments, step_m=0.6, min_clear=0.7):
    """贪心探索：每次挑"最开阔的方向"走一步，把场地扫开（用于自动建图）。"""
    spin_deg(node, 360.0, spin)
    travelled = 0.0
    for i in range(segments):
        psi, clear = best_heading(node)
        if clear < min_clear:
            node.get_logger().info(f"  四周最开阔方向也只有 {clear:.2f} m，结束探索（走了 {i} 段）")
            break
        d = (psi + 180.0) % 360.0 - 180.0  # 转到该方向（走最短一侧）
        if abs(d) > 8.0:
            spin_deg(node, d, spin)
        got = min(step_m, max(0.2, clear - clearance))
        node.get_logger().info(f"  第 {i + 1}/{segments} 段：朝向 {psi:+.0f}°（净空 {clear:.2f} m）前进 {got:.2f} m")
        move(node, 1.0, 0.0, got)
        travelled += got
        if i % 2 == 1:
            spin_deg(node, 360.0, spin)
    node.get_logger().info(f"探索结束，共前进 {travelled:.2f} m")
    spin_deg(node, 360.0, spin)


def spin_deg(node, deg, rate):
    """原地转 deg 度（+=逆时针）。"""
    sgn = 1.0 if deg >= 0 else -1.0
    yaw0 = node.yaw
    target = abs(math.radians(deg))
    t0 = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        done = abs(node.yaw - yaw0)
        if done >= target:
            break
        if time.time() - t0 > target / max(rate, 1e-3) * 2.5 + 8.0:
            node.get_logger().warn(f"  旋转超时（已转 {math.degrees(done):.0f}°）")
            break
        node.send(0.0, 0.0, sgn * rate)
    node.send(0.0, 0.0, 0.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.02)
    node.get_logger().info(f"  旋转 {math.degrees(node.yaw - yaw0):+.0f}° 完成")


def move(node, vx, vy, meters):
    """按本体速度方向走 meters 米（负数=反向）。"""
    norm = math.hypot(vx, vy)
    if norm < 1e-6:
        return
    ux, uy = vx / norm, vy / norm
    sgn = 1.0 if meters >= 0 else -1.0
    psi = math.degrees(math.atan2(uy * sgn, ux * sgn))  # 行驶方向（本体坐标系）
    x0, y0, yaw0 = node.x, node.y, node.yaw
    # 行驶方向转到 odom 坐标系（odom 原点=起点、+x=初始朝向；yaw0 是当前朝向）
    c, s = math.cos(yaw0), math.sin(yaw0)
    dir_x = ux * sgn * c - uy * sgn * s
    dir_y = ux * sgn * s + uy * sgn * c
    target = abs(meters)
    t0 = time.time()
    blocked = False
    along = 0.0
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        along = (node.x - x0) * dir_x + (node.y - y0) * dir_y
        if along >= target:
            break
        if time.time() - t0 > target / max(speed, 1e-3) * 3.0 + 8.0:
            node.get_logger().warn(f"  平移超时（已走 {along:.2f} m）")
            break
        clear = node.sector_min(psi)
        if clear < clearance:
            blocked = True
            node.get_logger().warn(f"  行驶方向净空 {clear:.2f} m < {clearance} m，提前停下（已走 {along:.2f} m）")
            break
        node.send(ux * speed, uy * speed, 0.0)
    node.send(0.0, 0.0, 0.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.02)
    node.get_logger().info(
        f"  平移 {along:+.2f} m 完成（净空 {node.sector_min(psi):.2f} m{'，被安全阈值截停' if blocked else ''}）"
    )
    return blocked


rclpy.init()
node = AutoDrive()
if not wait_odom(node):
    node.get_logger().error("❌ 收不到 /odom —— 桥接没连上 Isaac Sim？")
    rclpy.shutdown(); sys.exit(1)
node.get_logger().info(
    f"起点 odom=({node.x:+.2f}, {node.y:+.2f}) yaw={math.degrees(node.yaw):+.1f}°  安全净空={clearance} m"
)

try:
    if pattern == "map":
        node.get_logger().info("建图扫场：原地转 360° ×4，每次之间后退 0.5 m")
        spin_deg(node, 360.0, spin)
        for k in range(3):
            move(node, -1.0, 0.0, 0.5)   # 后退（远离起始位置的台面，往房间里走）
            spin_deg(node, 360.0, spin)
    elif pattern == "explore":
        node.get_logger().info("自动探索建图：每步挑最开阔的方向走 0.6 m（8 段）")
        explore(node, 8)
    elif dist is not None:
        move(node, 1.0, 0.0, dist)
    elif strafe is not None:
        move(node, 0.0, 1.0, strafe)
    elif turn is not None:
        spin_deg(node, turn, spin)
    elif sec:
        node.get_logger().info(f"定时指令 vx={vx} vy={vy} wz={wz} 持续 {sec}s")
        t0 = time.time()
        while time.time() - t0 < sec:
            node.send(vx, vy, wz)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.send(0.0, 0.0, 0.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
    else:
        node.get_logger().error("❌ 没给动作：用 --pattern map / --dist / --strafe / --turn / --vx..--sec")
        rclpy.shutdown(); sys.exit(2)

    node.get_logger().info(
        f"结束 odom=({node.x:+.2f}, {node.y:+.2f}) yaw={math.degrees(node.yaw):+.1f}°"
    )
finally:
    node.send(0.0, 0.0, 0.0)
    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()
PY
echo "[drive] 完成"
