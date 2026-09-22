#!/usr/bin/env python
"""SO101 主手只读探针：验证串口上的手臂是否活着（不写位置、不动电机）。

用的是 LeIsaac 遥操同一套 vendored 电机总线（leisaac.devices.lerobot.common.motors），
只做：连接 → 广播探测电机 ID → 读归一化关节位置 → 断开。

用法:
  python.sh -u /tmp/probe_leader.py /dev/ttyACM0 left_so101_leader.json
"""
import json
import os
import sys

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from leisaac.devices.lerobot.common.motors import (  # noqa: E402
    FeetechMotorsBus,
    Motor,
    MotorCalibration,
    MotorNormMode,
)

#: 主手标定文件所在目录（原来写死了开发机路径，赛队机器上必然找不到）。
#: 由本文件位置推导仓库根 → 不依赖仓库放在哪。
CACHE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    "source", "leisaac", "leisaac", "devices", "lerobot", ".cache",
)

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
cal_name = sys.argv[2] if len(sys.argv) > 2 else "left_so101_leader.json"

with open(f"{CACHE}/{cal_name}") as f:
    json_data = json.load(f)
# 与 SO101Leader._load_calibration 一致：JSON dict → MotorCalibration 对象
calibration = {name: MotorCalibration(**vals) for name, vals in json_data.items()}

motors = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

print(f"[probe] 端口={port} 标定文件={cal_name}")
bus = FeetechMotorsBus(port=port, motors=motors, calibration=calibration)
try:
    bus.connect()
    print("[probe] 已连接")
    cal = getattr(bus, "is_calibrated", None)
    print(f"[probe] is_calibrated = {cal() if callable(cal) else cal}")
    ping = bus.broadcast_ping()
    print(f"[probe] 广播探测到的电机 ID = {sorted(ping.keys()) if ping else ping}")
    pos = bus.sync_read("Present_Position")
    print("[probe] 归一化关节位置：")
    for k, v in pos.items():
        print(f"[probe]   {k:14s} = {float(v):+.4f}")
except Exception as e:  # noqa: BLE001
    import traceback

    traceback.print_exc()
    print(f"[probe] FAILED: {type(e).__name__}: {e}")
finally:
    try:
        bus.disconnect()
        print("[probe] 已断开")
    except Exception:  # noqa: BLE001
        pass

simulation_app.close()
