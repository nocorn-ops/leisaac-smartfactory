#!/usr/bin/env python
"""交互式标定 SO101 主手，把结果写进标定 JSON（会自动先备份原文件）。

为什么需要：主手的 ``Homing_Offset`` / 行程范围是**按手臂个体**存在电机里的；
如果标定文件与实际电机不一致（例如换过电机、装过舵机、或文件来自另一只手），
从手就会跟不准（方向/幅度不对）。重标一次即可。

用法（在仓库根目录、终端里跑；需要有人手动掰手臂）:

    env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV \\
      OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \\
      LD_LIBRARY_PATH="$PWD/reproduce/compat_libs_isaac6" \\
      <IsaacSim>/python.sh -u reproduce/calibrate_leader.py \\
        --port /dev/ttyACM0 --file left_so101_leader.json

脚本交互三步：
  1) 把整条手臂摆到各关节行程的**大致中间**位置，按回车；
  2) 依次把**每个关节**手动推过它的整个行程（脚本正在记录范围），推完按回车；
  3) 脚本把新的 homing offset / 行程范围写进标定文件。
"""
import argparse
import os
import shutil
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="交互式标定 SO101 主手")
parser.add_argument("--port", type=str, default="/dev/ttyACM0")
parser.add_argument(
    "--file",
    type=str,
    default="left_so101_leader.json",
    help="标定文件名（left_so101_leader.json / right_so101_leader.json / so101_leader.json）",
)
parser.add_argument("--no_backup", action="store_true", help="不备份原标定文件（默认会备份）")
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from leisaac.devices import SO101Leader  # noqa: E402

CACHE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "source", "leisaac", "leisaac", "devices", "lerobot", ".cache"
)
CACHE_DIR = os.path.normpath(CACHE_DIR)
cal_path = os.path.join(CACHE_DIR, args_cli.file)

if not os.path.exists(args_cli.port):
    print(f"[calib] 串口不存在: {args_cli.port}")
    simulation_app.close()
    raise SystemExit(2)

print(f"[calib] 端口={args_cli.port}  标定文件={cal_path}")
if os.path.exists(cal_path) and not args_cli.no_backup:
    backup = cal_path.replace(".json", f".bak-{time.strftime('%Y%m%d-%H%M%S')}.json")
    shutil.copy2(cal_path, backup)
    print(f"[calib] 已备份原标定 -> {backup}")

print("[calib] 即将开始交互式标定：")
print("        1) 把手臂摆到各关节行程的中间位置，按回车")
print("        2) 依次把每个关节推过它的整个行程，推完按回车")
try:
    SO101Leader(None, args_cli.port, True, args_cli.file)
    print(f"[calib] 完成，新标定已写入: {cal_path}")
except Exception as e:  # noqa: BLE001
    import traceback

    traceback.print_exc()
    print(f"[calib] FAILED: {type(e).__name__}: {e}")
finally:
    simulation_app.close()
