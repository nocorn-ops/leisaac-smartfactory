#!/usr/bin/env python3
"""ROS2 桥接客户端 — 连接 Isaac Sim TCP bridge，发布 /odom、/scan、/tf，订阅 /cmd_vel。

运行环境: ROS2 Humble (系统 Python)，不需要 Isaac Sim。
启动:
    source /opt/ros/humble/setup.bash
    python scripts/ros2_leisaac_bridge.py --sim_host 127.0.0.1 --sim_port 5560

发布 topics:
    /odom           (nav_msgs/Odometry)     — 里程计
    /scan           (sensor_msgs/LaserScan) — 激光扫描
    /tf             (tf2_msgs/TFMessage)    — odom → base_footprint

订阅 topics:
    /cmd_vel        (geometry_msgs/Twist)   — 底盘速度指令
"""

import argparse
import json
import math
import socket
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class LeisaacBridgeNode(Node):
    """ROS2 节点 — 桥接 Isaac Sim ↔ ROS2."""

    def __init__(self, sim_host: str, sim_port: int, rate_hz: float = 30.0):
        super().__init__("leisaac_bridge")
        self._sim_host = sim_host
        self._sim_port = sim_port
        self._rate_hz = rate_hz

        # ROS2 publishers
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        # ROS2 subscriber
        self._cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        # 复位：往 /leisaac/reset 发一条空消息 → 仿真底盘瞬移回初始位姿（建图完 → 导航前归位）
        self._reset_sub = self.create_subscription(Empty, "/leisaac/reset", self._reset_cb, 10)
        # ★ 运行中控制（2026-09-18）：让"仿真一直开着 → 键盘开过去 → 切换数采/推理"成立。
        #   /leisaac/arm_source : "device" | "policy" | "hold" | "auto"
        #                        也接受 JSON：{"arm_source":"policy","policy_port":5557}
        #   /leisaac/record     : "begin" | "success" | "fail" | "stop"
        #   例：ros2 topic pub --once /leisaac/arm_source std_msgs/String "{data: policy}"
        self._arm_src_sub = self.create_subscription(
            String, "/leisaac/arm_source", self._arm_source_cb, 10
        )
        self._record_sub = self.create_subscription(String, "/leisaac/record", self._record_cb, 10)
        self._pending_cmd = None
        self._pending_reset = False
        self._pending_control: list = []

        # 状态
        self._last_pose = None  # (x, y, yaw)
        self._pose_seq = 0
        self._scan_seq = 0
        # 是否真的收到了仿真发来的数据（用来判断"TCP 连上了但仿真没理我"）
        self.got_data = False
        self._start_time = self.get_clock().now()

        # TCP 连接
        self._sock = None
        self._sock_file = None

    def _cmd_vel_cb(self, msg: Twist):
        """订阅 /cmd_vel → 缓存待发送给 Isaac Sim.

        ★ 这里**绝对不要**每条都打日志：键盘节点按住键时以 **10 Hz** 发布，
        打一行就等于每秒刷 10 行（实测就是用户看到的"按一个键刷一堆日志"）。
        要知道底盘在不在动，看仿真窗口 HUD 或仿真侧的事件驱动日志就够了。
        """
        self._pending_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _arm_source_cb(self, msg: String):
        """上肢驱动源切换请求 → 缓存，下一拍发给仿真。"""
        data = (msg.data or "").strip()
        if not data:
            return
        if data.startswith("{"):
            self._pending_control.append(data)  # 已经是 JSON（可带 policy_port 等）
        else:
            self._pending_control.append(json.dumps({"arm_source": data}))

    def _record_cb(self, msg: String):
        """录制控制请求（begin/success/fail/stop）→ 缓存。"""
        data = (msg.data or "").strip()
        if data:
            self._pending_control.append(json.dumps({"record": data}))

    def _reset_cb(self, _msg: Empty):
        """收到 /leisaac/reset → 下一次刷写时通知仿真复位底盘。"""
        self._pending_reset = True
        self.get_logger().info("/leisaac/reset → 请求底盘回到初始位姿")

    def connect(self) -> bool:
        """连接 Isaac Sim bridge server."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(10.0)
            self._sock.connect((self._sim_host, self._sim_port))
            self._sock.settimeout(None)
            self._sock_file = self._sock.makefile("r", buffering=1)
            self.get_logger().info(f"Connected to Isaac Sim at {self._sim_host}:{self._sim_port}")
            # 发布静态 TF（只在初始化时发布一次）
            self._publish_static_tfs()
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return False

    def _publish_static_tfs(self):
        """发布静态 TF：base_footprint → base_link, base_lidar_link."""
        static_broadcaster = StaticTransformBroadcaster(self)
        now = self.get_clock().now().to_msg()
        transforms = []

        # base_footprint → base_link (重合)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)

        # base_footprint → base_lidar_link (LiDAR 在底盘上方 25cm)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "base_lidar_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.25
        t.transform.rotation.w = 1.0
        transforms.append(t)

        static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Static TFs published: base_footprint → base_link, base_lidar_link")

    def flush_cmd_vel(self):
        """发送缓存的 cmd_vel（以及复位请求）到 Isaac Sim."""
        if self._sock is None:
            return
        if self._pending_reset:
            try:
                self._sock.sendall((json.dumps({"reset": True}) + "\n").encode("utf-8"))
                self._pending_reset = False
            except Exception:
                pass
        if self._pending_cmd is not None:
            try:
                msg = json.dumps({"cmd_vel": list(self._pending_cmd)}) + "\n"
                self._sock.sendall(msg.encode("utf-8"))
                self._pending_cmd = None
            except Exception:
                pass
        while self._pending_control:
            try:
                line = self._pending_control[0]
                self._sock.sendall((line + "\n").encode("utf-8"))
                self._pending_control.pop(0)
            except Exception:
                break  # 发不出去就先留着，别丢

    def read_messages(self):
        """读取 Isaac Sim 发来的消息并发布到 ROS2（非阻塞）."""
        if self._sock_file is None:
            return
        try:
            # 设置 socket 为非阻塞，只读已有数据
            self._sock.setblocking(False)
            while True:
                line = self._sock_file.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self.got_data = True
                msg_type = msg.get("type")
                data = msg.get("data", {})
                if msg_type == "pose":
                    self._publish_odom_and_tf(data)
                elif msg_type == "scan":
                    self._publish_scan(data)
                elif "error" in msg:
                    # ★ 冲突检测：仿真侧同一时刻只服务一个桥接客户端。
                    #   被拒时它会把原因发过来 —— 这里必须**大声报出来**，
                    #   否则表现就是"键盘能按、车不动"，非常难查（真踩过）。
                    self.get_logger().error(
                        "❌ 仿真侧拒绝了本客户端："
                        f"{msg.get('error')} —— {msg.get('hint', '')}\n"
                        "   原因：同一时刻只允许一个桥接客户端（已经有一个在连）。\n"
                        "   处理：关掉多余的键盘/建图/导航/复位进程，一个容器里只留一个；"
                        "自查 `bash reproduce/stop_all.sh --status`（宿主机）"
                    )
        except (BlockingIOError, socket.timeout):
            pass
        except Exception as e:
            pass  # 连接断开等，忽略
        finally:
            if self._sock:
                self._sock.setblocking(True)

    def _publish_odom_and_tf(self, data: list):
        """发布 /odom 和 tf."""
        x, y, yaw = data[0], data[1], data[2]
        now = self.get_clock().now().to_msg()

        # TF: odom → base_footprint
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = math.sin(yaw / 2.0)
        tf_msg.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_broadcaster.sendTransform(tf_msg)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # 直接发布 odom 作为 filtered（仿真中 odom 足够精准）
        self._odom_pub.publish(odom)
        self._pose_seq += 1

    def _publish_scan(self, data: dict):
        """发布 /scan (LaserScan)."""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_lidar_link"
        scan.angle_min = float(data.get("angle_min", -math.pi))
        scan.angle_max = float(data.get("angle_max", math.pi))
        scan.angle_increment = float(data.get("angle_increment", math.pi / 180.0))
        scan.range_min = float(data.get("range_min", 0.2))
        scan.range_max = float(data.get("range_max", 8.0))
        scan.ranges = [float(r) for r in data.get("ranges", [])]
        self._scan_pub.publish(scan)
        self._scan_seq += 1

    def disconnect(self):
        """断开连接."""
        if self._sock_file:
            try:
                self._sock_file.close()
            except Exception:
                pass
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self.get_logger().info("Disconnected from Isaac Sim")


def main():
    parser = argparse.ArgumentParser(description="ROS2 ↔ Isaac Sim bridge client")
    parser.add_argument("--sim_host", default="127.0.0.1", help="Isaac Sim bridge server host")
    parser.add_argument("--sim_port", type=int, default=5560, help="Isaac Sim bridge server port")
    parser.add_argument("--rate_hz", type=float, default=30.0, help="Bridge update rate")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = LeisaacBridgeNode(sim_host=args.sim_host, sim_port=args.sim_port, rate_hz=args.rate_hz)

    if not node.connect():
        node.get_logger().error("Exiting — cannot connect to Isaac Sim bridge")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    period = 1.0 / args.rate_hz
    t_start = time.time()
    warned = False
    # ★ 冲突检测：`/cmd_vel` 上多于一个发布者 = 两边抢方向盘（键控 + Nav2 同时发指令），
    #   表现是机器人"抖动/走不动/来回画龙"。这里只看不拦，把发布者名字打出来让人自己关。
    t_pub_check = 0.0
    last_pub_count = 0
    try:
        while rclpy.ok():
            # 处理 ROS2 回调
            rclpy.spin_once(node, timeout_sec=0.001)
            # 读取 Isaac Sim 消息
            node.read_messages()
            # ⚠️ 仿真侧的 SimBridgeServer 同一时刻只服务**一个**客户端：多开的那个桥接
            #    TCP 能连上（内核完成三次握手），但仿真永远不会读它的数据 —— 表现就是
            #    "键盘/导航都能发指令，机器人就是不动"。这里给一个明确的提示。
            if not warned and not node.got_data and time.time() - t_start > 6.0:
                warned = True
                node.get_logger().error(
                    "❌ 连上了 TCP 但 6 秒内没收到任何位姿/激光数据 —— 极可能有**另一个桥接客户端**"
                    "（ros2_leisaac_bridge.py）已经连着仿真了。同一时刻只能有一个："
                    "请把多余的关掉（含 ros2_mapping.sh / ros2_navigation.sh / "
                    "ros2_keyboard_container.sh 起的那些），只留一个。"
                )
            # ★ 每 2 秒看一眼 /cmd_vel 有几个发布者（只在数量变化时提示，避免刷屏）
            if time.time() - t_pub_check > 2.0:
                t_pub_check = time.time()
                try:
                    n_pub = node.count_publishers("/cmd_vel")
                except Exception:  # noqa: BLE001
                    n_pub = last_pub_count
                if n_pub != last_pub_count:
                    if n_pub > 1:
                        try:
                            names = [
                                f"{e.node_name}"
                                for e in node.get_publishers_info_by_topic("/cmd_vel")
                            ]
                        except Exception:  # noqa: BLE001
                            names = []
                        node.get_logger().warn(
                            f"⚠ /cmd_vel 上有 {n_pub} 个发布者 {names} —— 两个指令源会互相抢方向盘"
                            "（典型：teleop_twist_keyboard 和 Nav2 同时开）。"
                            "请只留一个：手动遥控时先 `ros2 lifecycle set /bt_navigator deactivate`，"
                            "或把 Nav2 的 `controller_server` 停掉。"
                        )
                    last_pub_count = n_pub
            # 发送 cmd_vel
            node.flush_cmd_vel()
            time.sleep(period)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
