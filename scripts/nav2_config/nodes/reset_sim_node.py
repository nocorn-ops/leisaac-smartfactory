#!/usr/bin/env python3
"""底盘归位节点 —— 把 `reproduce/ros2_reset_sim.sh` 里内嵌的 Python 提成正式节点。

发一条 ``/leisaac/reset`` → 仿真侧底盘瞬移回初始位姿，并等 ``/odom`` 真的回到 (0,0,0)。
建图走完一圈后先存图、再归位，然后就能直接起导航
（Nav2 的 AMCL / 固定 map→odom 都假设机器人在 map 原点、朝向 map +x）。

参数:
    timeout_sec   等归位的超时（默认 12.0）
    xy_tol        位置容差 m（默认 0.05）
    yaw_tol_deg   朝向容差 度（默认 5.0）
    warmup_sec    发复位前先空转多久建立连接（默认 1.5）

退出码: 0 = 已归位, 1 = 收不到 /odom 或超时未归位
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Empty

#: ★ 浮点参数一律用 dynamic_typing 声明，避免 ``timeout_sec:=4``（整数）被判成 INTEGER 而崩。
_FLOAT = ParameterDescriptor(dynamic_typing=True)


class ResetSim(Node):
    def __init__(self):
        super().__init__("leisaac_reset_sim")

        self.declare_parameter("timeout_sec", 12.0, _FLOAT)
        self.declare_parameter("xy_tol", 0.05, _FLOAT)
        self.declare_parameter("yaw_tol_deg", 5.0, _FLOAT)
        self.declare_parameter("warmup_sec", 1.5, _FLOAT)

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.xy_tol = float(self.get_parameter("xy_tol").value)
        self.yaw_tol = math.radians(float(self.get_parameter("yaw_tol_deg").value))
        self.warmup_sec = float(self.get_parameter("warmup_sec").value)

        self.pub = self.create_publisher(Empty, "/leisaac/reset", 10)
        self.odom = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, msg):
        self.odom = msg

    def _pose(self):
        m = self.odom
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)
        return x, y, yaw

    def run(self) -> int:
        # 先空转一会，确保订阅/publisher 都已建立连接
        t0 = time.time()
        while time.time() - t0 < self.warmup_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.odom is None:
            self.get_logger().error("❌ 收不到 /odom —— 桥接没连上 Isaac Sim？")
            return 1

        self.pub.publish(Empty())
        self.get_logger().info("已发送复位请求 → 等底盘回到初始位姿 ...")

        t0 = time.time()
        while time.time() - t0 < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is None:
                continue
            x, y, yaw = self._pose()
            if abs(x) < self.xy_tol and abs(y) < self.xy_tol and abs(yaw) < self.yaw_tol:
                self.get_logger().info(
                    f"✅ 已归位：odom=({x:+.3f}, {y:+.3f}) yaw={math.degrees(yaw):+.1f}°"
                    f"（用时 {time.time() - t0:.1f}s）"
                )
                return 0
            time.sleep(0.2)

        x, y, yaw = self._pose()
        self.get_logger().warn(
            f"⚠️ {self.timeout_sec:.0f} 秒内没回到起点，当前 odom=({x:+.3f}, {y:+.3f}) "
            f"yaw={math.degrees(yaw):+.1f}°"
        )
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = ResetSim()
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
