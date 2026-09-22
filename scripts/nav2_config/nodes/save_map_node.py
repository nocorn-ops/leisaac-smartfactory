#!/usr/bin/env python3
"""存图节点 —— **照 duojin01 的做法**：调 slam_toolbox 自带的 ``/slam_toolbox/save_map`` 服务。

duojin01 用的是 C++ 的 `duojin01_slam_tools/src/save_map_client_node.cpp`，逻辑就是：
    解析输出目录 → 自动数字命名（最大数字 + 1）→ 等 ``/slam_toolbox/save_map`` →
    用**绝对路径前缀**当 ``name`` 发请求 → 检查返回码。这里用 Python 复刻同样的语义。

为什么不用 ``nav2_map_server map_saver_cli``（原来的做法）:
    ``map_saver_cli`` 自己去订阅 ``/map``，QoS 得猜；而 slam 起来但还没产出地图时它会以
    ``Failed to spin map subscription`` 失败（实测踩过），且失败原因不明确。
    服务方式由 slam_toolbox 自己序列化，返回码能区分"还没收到地图"和"真失败"；
    leisaac 的 ``slam_toolbox.yaml`` 里本来就开着 ``use_map_saver: true``，服务是现成的（已实测）。

参数:
    map_name      地图名；留空或 ``auto`` = **自动递增**（0 → 1 → 2 …，不覆盖历史地图）
    output_dir    输出目录（默认脚本同级的 ../maps，即 scripts/nav2_config/maps）
    wait_timeout  等服务和等存完的超时 s（默认 30.0）

退出码: 0 = 存图成功, 1 = 服务没来 / slam 还没收到地图 / 存图失败
"""

import collections
import os
import sys

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from slam_toolbox.srv import SaveMap

# 允许 `python3 nodes/save_map_node.py` 直接运行时 import 到上级目录的 map_paths
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from map_paths import resolve_map_name  # noqa: E402

_DEFAULT_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "maps"
)


def pgm_stats(path: str) -> str:
    """读 PGM 头 + 像素，返回统计字符串（像素值：0=障碍 205=未知 254=空闲）。"""
    with open(path, "rb") as f:
        data = f.read()

    # 跳过 PGM 头（P5 + 空白分隔的 3 个数字，允许 # 注释）
    toks, i = [], 0
    while len(toks) < 4:
        while data[i : i + 1].isspace():
            i += 1
        if data[i : i + 1] == b"#":
            while data[i : i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j : j + 1].isspace():
            j += 1
        toks.append(data[i:j])
        i = j

    w, h = int(toks[1]), int(toks[2])
    px = data[i + 1 :]
    cnt = collections.Counter(px)
    occ = sum(v for k, v in cnt.items() if k < 100)
    free = sum(v for k, v in cnt.items() if v and k > 240)
    total = max(w * h, 1)
    return (
        f"      尺寸 {w}x{h} = {w * h} 像素 | 障碍 {occ} ({occ / total * 100:.1f}%) "
        f"空闲 {free} ({free / total * 100:.1f}%)"
    )


class SaveMapNode(Node):
    def __init__(self):
        super().__init__("leisaac_save_map")

        self.declare_parameter("map_name", "auto")
        self.declare_parameter("output_dir", _DEFAULT_OUTPUT_DIR)
        self.declare_parameter("wait_timeout", 30.0, ParameterDescriptor(dynamic_typing=True))

        self.output_dir = str(self.get_parameter("output_dir").value)
        self.wait_timeout = float(self.get_parameter("wait_timeout").value)
        self.name = resolve_map_name(str(self.get_parameter("map_name").value), self.output_dir)
        self.client = self.create_client(SaveMap, "/slam_toolbox/save_map")

    def result_label(self, code: int) -> str:
        if code == SaveMap.Response.RESULT_SUCCESS:
            return "SUCCESS"
        if code == SaveMap.Response.RESULT_NO_MAP_RECEIEVD:
            return "还没收到地图（slam_toolbox 起来了吗？机器人走过一圈了吗？）"
        if code == SaveMap.Response.RESULT_UNDEFINED_FAILURE:
            return "未定义失败"
        return f"未知返回码 {code}"

    def run(self) -> int:
        os.makedirs(self.output_dir, exist_ok=True)
        prefix = os.path.join(self.output_dir, self.name)

        self.get_logger().info(
            f"等 /slam_toolbox/save_map 服务 ...（超时 {self.wait_timeout:.0f}s）"
        )
        if not self.client.wait_for_service(timeout_sec=self.wait_timeout):
            self.get_logger().error(
                "❌ 等不到 /slam_toolbox/save_map —— 建图进程起了吗？"
                "（本入口默认 bridge:=false，是插进已有 ROS2 图里用的）"
            )
            return 1

        self.get_logger().info(f"保存地图到 {prefix}.{{pgm,yaml}} ...")
        request = SaveMap.Request()
        request.name.data = prefix
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.wait_timeout)

        response = future.result()
        if response is None:
            self.get_logger().error(f"❌ {self.wait_timeout:.0f} 秒内没等到存图结果")
            return 1
        if response.result != SaveMap.Response.RESULT_SUCCESS:
            self.get_logger().error(
                f"❌ 存图失败：{self.result_label(response.result)}"
                f"（返回码 {response.result}）"
            )
            return 1

        self.get_logger().info("✅ slam_toolbox 已保存")
        ok = True
        for suffix in (".pgm", ".yaml"):
            path = prefix + suffix
            if os.path.isfile(path) and os.path.getsize(path) > 0:
                self.get_logger().info(f"  ✅ {path} ({os.path.getsize(path)} 字节)")
            else:
                self.get_logger().error(f"  ❌ {path} 缺失/为空")
                ok = False

        if ok:
            try:
                self.get_logger().info("地图统计（像素值：0=障碍 205=未知 254=空闲）:")
                self.get_logger().info(pgm_stats(prefix + ".pgm"))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"  （统计失败：{exc}）")
        return 0 if ok else 1


def main(args=None):
    rclpy.init(args=args)
    node = SaveMapNode()
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
