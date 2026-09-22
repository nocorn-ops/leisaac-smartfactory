#!/usr/bin/env python3
"""发目标点节点 —— 把 `reproduce/ros2_send_goal.sh` 里内嵌的 Python 提成正式节点。

在 map 坐标系里给 Nav2 一个目标点，等它走完并打印实时进度（剩余距离 / 恢复行为次数）。

★ 关键：**必须等 ``bt_navigator`` 生命周期进入 ACTIVE 再发**。
  只等 ``/navigate_to_pose`` 动作服务器"出现"是不够的 —— 动作服务器在 bt_navigator
  节点一启动就存在，但那时它还只是 unconfigured/inactive，目标会被**直接拒绝**（实测踩过：
  launch 起来 3 秒就发，bt_navigator 回 "goal rejected"）。
  原来的 `ros2_navigation.sh` 用 ``ros2 lifecycle get /bt_navigator`` 轮询到 active 才发，
  这里用等价的 ``/bt_navigator/get_state`` 服务，纯 rclpy，不依赖 ros2 CLI。

参数:
    x, y        map 坐标系目标位置 m
    yaw_deg     目标朝向 度（默认 0）
    wait_sec    等动作服务器 + 等 bt_navigator active 的总超时（默认 90.0）

退出码: 0 = SUCCEEDED, 1 = 失败/被中止/等不到 active
"""

import math
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node

#: ★ 浮点参数一律用 dynamic_typing 声明，避免 ``yaw_deg:=0``（整数）被判成 INTEGER 而崩。
_FLOAT = ParameterDescriptor(dynamic_typing=True)


class GoalClient(Node):
    def __init__(self):
        super().__init__("leisaac_send_goal")

        self.declare_parameter("x", 0.0, _FLOAT)
        self.declare_parameter("y", 0.0, _FLOAT)
        self.declare_parameter("yaw_deg", 0.0, _FLOAT)
        self.declare_parameter("wait_sec", 90.0, _FLOAT)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = math.radians(float(self.get_parameter("yaw_deg").value))
        self.wait_sec = float(self.get_parameter("wait_sec").value)

        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self._t0 = time.time()
        self._last_feedback = 0.0

    def _on_feedback(self, msg):
        fb = msg.feedback
        now = time.time()
        if now - self._last_feedback > 1.0:
            self._last_feedback = now
            self.get_logger().info(
                f"  前进中 {now - self._t0:5.1f}s  剩余 {fb.distance_remaining:5.2f} m  "
                f"恢复行为 {fb.number_of_recoveries} 次"
            )

    def wait_until_active(self, deadline: float) -> bool:
        """轮询 /bt_navigator/get_state，直到它 ACTIVE（= Nav2 真的能接目标了）。"""
        if not self.state_client.wait_for_service(timeout_sec=max(0.0, deadline - time.time())):
            self.get_logger().error("❌ 等不到 /bt_navigator/get_state 服务 —— Nav2 没起来？")
            return False

        last_state = None
        while time.time() < deadline:
            future = self.state_client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            result = future.result()
            if result is not None:
                state = result.current_state
                label = state.label or str(state.id)
                if state.id == State.PRIMARY_STATE_ACTIVE:
                    self.get_logger().info(f"bt_navigator 已 ACTIVE（用时 {time.time() - self._t0:.1f}s）")
                    return True
                if label != last_state:
                    self.get_logger().info(f"  等 bt_navigator 激活… 当前 {label}")
                    last_state = label
            time.sleep(0.5)

        self.get_logger().error(
            f"❌ {self.wait_sec:.0f} 秒内 bt_navigator 没进入 ACTIVE（当前 {last_state}）"
            f" —— 看 Nav2 日志（常见原因：没有 /scan 或 TF 不全，代价地图起不来）"
        )
        return False

    def run(self) -> int:
        self.get_logger().info(
            f"目标: map 坐标系 ({self.x:+.2f}, {self.y:+.2f}) yaw={math.degrees(self.yaw):+.0f}°"
        )
        deadline = time.time() + self.wait_sec

        self.get_logger().info("等待 /navigate_to_pose 动作服务器 ...")
        if not self.client.wait_for_server(timeout_sec=self.wait_sec):
            self.get_logger().error(
                f"❌ {self.wait_sec:.0f} 秒内没等到 /navigate_to_pose —— Nav2 没起来？"
            )
            return 1
        if not self.wait_until_active(deadline):
            return 1

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self._t0 = time.time()
        self.get_logger().info("已发送目标点，等待结果 ...")
        send = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=20.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("❌ 目标点被拒绝（bt_navigator 处于 active 吗？）")
            return 1

        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result)
        status = result.result().status
        names = {2: "UNKNOWN", 3: "ACCEPTED", 4: "SUCCEEDED ✅", 5: "CANCELED", 6: "ABORTED ❌"}
        self.get_logger().info(
            f"结果: {names.get(status, status)}  用时 {time.time() - self._t0:.1f}s"
        )
        return 0 if status == 4 else 1


def main(args=None):
    rclpy.init(args=args)
    node = GoalClient()
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
