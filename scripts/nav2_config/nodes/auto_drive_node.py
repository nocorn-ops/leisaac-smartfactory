#!/usr/bin/env python3
"""自动行驶节点 —— 把 `reproduce/ros2_auto_drive.sh` 里内嵌的 Python 提成正式节点。

用 ``/odom`` 闭环（走够多少米/转够多少度就停），并用 ``/scan`` 检查行驶方向净空：
净空 < ``clearance`` 就提前停下（运动学底盘会穿墙，靠这层兜住）。

被 `scripts/nav2_config/ops/auto_drive.launch.py` 启动，也被 ``ops/mapping.launch.py auto:=true``
当作建图扫场用。逻辑与 2026-09-15 已验证的 shell 版本逐行一致，只把 argv 换成了 ROS 参数。

参数（都可通过 ``-p 名字:=值`` 覆盖）:
    pattern     none | explore | map   （none + 其它动作全为 0 时报错，避免空跑）
    dist        直行距离 m（负数=后退）
    strafe      左平移距离 m（负数=右）
    turn        原地转角 度（负数=顺时针）
    vx/vy/wz    原始速度；配合 sec 使用
    sec         定时指令持续秒数
    clearance   安全净空 m（默认 0.35）
    speed       平移速度 m/s（默认 0.20）
    spin        旋转角速度 rad/s（默认 0.60）
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

#: ★ 浮点参数一律用 dynamic_typing 声明：否则用户写 ``turn:=90``（整数）会被 ROS 判成
#:   INTEGER，而声明是 DOUBLE → 启动即 InvalidParameterTypeException（踩过）。
_FLOAT = ParameterDescriptor(dynamic_typing=True)


class AutoDrive(Node):
    def __init__(self):
        super().__init__("leisaac_auto_drive")

        self.declare_parameter("pattern", "none")
        self.declare_parameter("dist", 0.0, _FLOAT)
        self.declare_parameter("strafe", 0.0, _FLOAT)
        self.declare_parameter("turn", 0.0, _FLOAT)
        self.declare_parameter("vx", 0.0, _FLOAT)
        self.declare_parameter("vy", 0.0, _FLOAT)
        self.declare_parameter("wz", 0.0, _FLOAT)
        self.declare_parameter("sec", 0.0, _FLOAT)
        self.declare_parameter("clearance", 0.35, _FLOAT)
        self.declare_parameter("speed", 0.20, _FLOAT)
        self.declare_parameter("spin", 0.60, _FLOAT)

        self.pattern = str(self.get_parameter("pattern").value)
        self.dist = float(self.get_parameter("dist").value)
        self.strafe = float(self.get_parameter("strafe").value)
        self.turn = float(self.get_parameter("turn").value)
        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.wz = float(self.get_parameter("wz").value)
        self.sec = float(self.get_parameter("sec").value)
        self.clearance = float(self.get_parameter("clearance").value)
        self.speed = float(self.get_parameter("speed").value)
        self.spin = float(self.get_parameter("spin").value)

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

        self.x = None
        self.y = None
        self.yaw = None
        self.scan = None
        self._yaw_raw = None
        self._yaw_cont = 0.0

    # ── 回调 ──────────────────────────────────────────────────────────────
    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        y = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        if self._yaw_raw is not None:
            d = y - self._yaw_raw
            d = (d + math.pi) % (2 * math.pi) - math.pi
            self._yaw_cont += d
        self._yaw_raw = y
        self.yaw = self._yaw_cont

    def _on_scan(self, msg):
        self.scan = msg

    # ── 工具 ──────────────────────────────────────────────────────────────
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

    def wait_odom(self, timeout=15.0):
        t0 = time.time()
        while self.x is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.x is not None

    def best_heading(self, half=25.0, n=72):
        """视野最开阔的行驶方向（本体坐标系角度, 该方向净空）。"""
        best = (0.0, -1.0)
        for k in range(n):
            psi = -180.0 + 360.0 * k / n
            c = self.sector_min(psi, half)
            if c > best[1]:
                best = (psi, c)
        return best

    # ── 动作 ──────────────────────────────────────────────────────────────
    def spin_deg(self, deg, rate):
        """原地转 deg 度（+=逆时针）。"""
        sgn = 1.0 if deg >= 0 else -1.0
        yaw0 = self.yaw
        target = abs(math.radians(deg))
        t0 = time.time()
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            done = abs(self.yaw - yaw0)
            if done >= target:
                break
            if time.time() - t0 > target / max(rate, 1e-3) * 2.5 + 8.0:
                self.get_logger().warn(f"  旋转超时（已转 {math.degrees(done):.0f}°）")
                break
            self.send(0.0, 0.0, sgn * rate)
        self.send(0.0, 0.0, 0.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().info(f"  旋转 {math.degrees(self.yaw - yaw0):+.0f}° 完成")

    def move(self, vx, vy, meters):
        """按本体速度方向走 meters 米（负数=反向）。返回是否被安全阈值截停。"""
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            return False
        ux, uy = vx / norm, vy / norm
        sgn = 1.0 if meters >= 0 else -1.0
        psi = math.degrees(math.atan2(uy * sgn, ux * sgn))  # 行驶方向（本体坐标系）
        x0, y0, yaw0 = self.x, self.y, self.yaw
        # 行驶方向转到 odom 坐标系（odom 原点=起点、+x=初始朝向；yaw0 是当前朝向）
        c, s = math.cos(yaw0), math.sin(yaw0)
        dir_x = ux * sgn * c - uy * sgn * s
        dir_y = ux * sgn * s + uy * sgn * c
        target = abs(meters)
        t0 = time.time()
        blocked = False
        along = 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            along = (self.x - x0) * dir_x + (self.y - y0) * dir_y
            if along >= target:
                break
            if time.time() - t0 > target / max(self.speed, 1e-3) * 3.0 + 8.0:
                self.get_logger().warn(f"  平移超时（已走 {along:.2f} m）")
                break
            clear = self.sector_min(psi)
            if clear < self.clearance:
                blocked = True
                self.get_logger().warn(
                    f"  行驶方向净空 {clear:.2f} m < {self.clearance} m，提前停下（已走 {along:.2f} m）"
                )
                break
            self.send(ux * self.speed, uy * self.speed, 0.0)
        self.send(0.0, 0.0, 0.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().info(
            f"  平移 {along:+.2f} m 完成（净空 {self.sector_min(psi):.2f} m"
            f"{'，被安全阈值截停' if blocked else ''}）"
        )
        return blocked

    def explore(self, segments, step_m=0.6, min_clear=0.7):
        """贪心探索：每次挑"最开阔的方向"走一步，把场地扫开（用于自动建图）。"""
        self.spin_deg(360.0, self.spin)
        travelled = 0.0
        for i in range(segments):
            psi, clear = self.best_heading()
            if clear < min_clear:
                self.get_logger().info(f"  四周最开阔方向也只有 {clear:.2f} m，结束探索（走了 {i} 段）")
                break
            d = (psi + 180.0) % 360.0 - 180.0  # 转到该方向（走最短一侧）
            if abs(d) > 8.0:
                self.spin_deg(d, self.spin)
            got = min(step_m, max(0.2, clear - self.clearance))
            self.get_logger().info(
                f"  第 {i + 1}/{segments} 段：朝向 {psi:+.0f}°（净空 {clear:.2f} m）前进 {got:.2f} m"
            )
            self.move(1.0, 0.0, got)
            travelled += got
            if i % 2 == 1:
                self.spin_deg(360.0, self.spin)
        self.get_logger().info(f"探索结束，共前进 {travelled:.2f} m")
        self.spin_deg(360.0, self.spin)


def run(node: AutoDrive) -> int:
    """按参数选动作跑一遍，返回退出码（0 正常）。"""
    if not node.wait_odom():
        node.get_logger().error("❌ 收不到 /odom —— 桥接没连上 Isaac Sim？")
        return 1

    node.get_logger().info(
        f"起点 odom=({node.x:+.2f}, {node.y:+.2f}) yaw={math.degrees(node.yaw):+.1f}°  "
        f"安全净空={node.clearance} m"
    )

    if node.pattern == "map":
        node.get_logger().info("建图扫场：原地转 360° ×4，每次之间后退 0.5 m")
        node.spin_deg(360.0, node.spin)
        for _ in range(3):
            node.move(-1.0, 0.0, 0.5)  # 后退（远离起始位置的台面，往房间里走）
            node.spin_deg(360.0, node.spin)
    elif node.pattern == "explore":
        node.get_logger().info("自动探索建图：每步挑最开阔的方向走 0.6 m（8 段）")
        node.explore(8)
    elif node.dist:
        node.move(1.0, 0.0, node.dist)
    elif node.strafe:
        node.move(0.0, 1.0, node.strafe)
    elif node.turn:
        node.spin_deg(node.turn, node.spin)
    elif node.sec:
        node.get_logger().info(f"定时指令 vx={node.vx} vy={node.vy} wz={node.wz} 持续 {node.sec}s")
        t0 = time.time()
        while time.time() - t0 < node.sec:
            node.send(node.vx, node.vy, node.wz)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.send(0.0, 0.0, 0.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.02)
    else:
        node.get_logger().error(
            "❌ 没给动作：用 pattern:=map / pattern:=explore / dist:=… / strafe:=… / "
            "turn:=… / vx:=… sec:=…"
        )
        return 2

    node.get_logger().info(
        f"结束 odom=({node.x:+.2f}, {node.y:+.2f}) yaw={math.degrees(node.yaw):+.1f}°"
    )
    return 0


def main(args=None):
    rclpy.init(args=args)
    node = AutoDrive()
    code = 1
    try:
        code = run(node)
    finally:
        node.send(0.0, 0.0, 0.0)
        time.sleep(0.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
