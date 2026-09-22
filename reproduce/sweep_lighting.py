#!/usr/bin/env python
"""A-1：扫灯光强度，找能让三路相机亮度对齐训练数据（front≈56）的档位。

背景：训练数据 `kitchen_biarm.hdf5` 的 obs 三路相机均值 front=55.9 / left_wrist=28.0 /
right_wrist=19.3，我们在 6.0 下渲出来是 142.0 / 150.4 / 103.5（亮 2~7 倍）。
取景/构图已确认一致（见 HANDOFF §6.6-5），所以是光照/曝光问题。

做法：同一个进程里改 DomeLight 的 intensity，每档渲几帧后读三路相机均值。
用法: reproduce/sweep_lighting.py --out /tmp/light
"""
import argparse
import dataclasses
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--out", type=str, default="/tmp/light")
parser.add_argument("--intensities", type=float, nargs="*", default=[3000.0, 2000.0, 1200.0, 700.0, 400.0, 200.0, 100.0])
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402

CAMS = ("front", "left_wrist", "right_wrist")
# 训练数据（demo_0 第 0 帧）三路均值，作为目标
TARGET = {"front": 55.9, "left_wrist": 28.0, "right_wrist": 19.3}

env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.use_teleop_device(get_task_type(args_cli.task))
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()
for _ in range(60):
    env.sim.step(render=True)
    env.scene.update(env.physics_dt)

import omni.usd  # noqa: E402
from pxr import Usd, UsdLux  # noqa: E402

stage = omni.usd.get_context().get_stage()

# 先列出场景里所有灯光及其强度，确认到底是谁在照亮
print("[light] 场景灯光：")
for prim in stage.Traverse():
    if prim.GetTypeName() in ("DomeLight", "DistantLight", "SphereLight", "RectLight", "DiskLight"):
        i = prim.GetAttribute("inputs:intensity")
        print(f"  {prim.GetPath().pathString} type={prim.GetTypeName()} intensity={i.Get() if i and i.IsValid() else None}")

light_prim = stage.GetPrimAtPath("/World/envs/env_0/Light")
if not light_prim or not light_prim.IsValid():
    raise SystemExit("[light] 找不到 /World/envs/env_0/Light")
light = UsdLux.DomeLight(light_prim)
attr = light.GetIntensityAttr()
print(f"[light] 原始 intensity = {attr.Get()}")

os.makedirs(args_cli.out, exist_ok=True)


def capture_means() -> dict:
    for _ in range(12):  # 让 RTX 出几帧稳定画面
        env.sim.render()
        env.scene.update(env.physics_dt)
    out = {}
    for cam in CAMS:
        sensor = env.scene.sensors[cam]
        arr = sensor.data.output["rgb"][0]
        arr = arr.torch if hasattr(arr, "torch") else arr
        arr = arr.detach().cpu().numpy()[..., :3]
        out[cam] = float(arr.mean())
    return out


print(f"\n{'intensity':>10} | " + " | ".join(f"{c:>12}" for c in CAMS) + " |  与目标差(front)")
rows = []
for val in args_cli.intensities:
    attr.Set(float(val))
    means = capture_means()
    diff = means["front"] - TARGET["front"]
    rows.append((val, means))
    print(f"{val:>10.0f} | " + " | ".join(f"{means[c]:>12.1f}" for c in CAMS) + f" |  {diff:+.1f}")

best = min(rows, key=lambda r: abs(r[1]["front"] - TARGET["front"]))
print(f"\n[light] 最接近目标的一档：intensity={best[0]:.0f} → " + ", ".join(f"{c}={best[1][c]:.1f}" for c in CAMS))
print(f"[light] 目标（训练数据）：" + ", ".join(f"{c}={TARGET[c]:.1f}" for c in CAMS))

# 存几张对比图
attr.Set(float(best[0]))
for _ in range(12):
    env.sim.render()
    env.scene.update(env.physics_dt)
from PIL import Image  # noqa: E402

for cam in CAMS:
    arr = env.scene.sensors[cam].data.output["rgb"][0]
    arr = arr.torch if hasattr(arr, "torch") else arr
    arr = arr.detach().cpu().numpy()[..., :3].astype(np.uint8)
    Image.fromarray(arr).save(os.path.join(args_cli.out, f"best_{cam}.png"))
print(f"[light] 已存 {args_cli.out}/best_*.png（intensity={best[0]:.0f}）")
attr.Set(3000.0)

env.close()
simulation_app.close()
