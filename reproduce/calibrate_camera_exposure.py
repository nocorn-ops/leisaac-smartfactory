#!/usr/bin/env python
"""A-1 第二步：只调**相机自己的曝光**（UsdGeom.Camera.exposure，单位 stops），不动场景灯光，
把三路相机的亮度压到训练数据的水平（front 55.9 / left_wrist 28.0 / right_wrist 19.3）。

这样 GUI 视口（默认相机）的观感不变，只有"录进数据集/喂给策略"的那三路相机变暗。
用法: reproduce/calibrate_camera_exposure.py --out /tmp/camcal
"""
import argparse
import dataclasses
import math
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--out", type=str, default="/tmp/camcal")
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
from pxr import UsdGeom  # noqa: E402

stage = omni.usd.get_context().get_stage()
os.makedirs(args_cli.out, exist_ok=True)


def prim_of(cam: str) -> str:
    raw = str(env.scene.sensors[cam].cfg.prim_path)
    # 解析后的正则/宏形式都换成 env_0
    import re

    raw = raw.replace("{ENV_REGEX_NS}", "/World/envs/env_0")
    raw = re.sub(r"^/World/envs/env_\[\^/\]\+/", "/World/envs/env_0/", raw)
    return re.sub(r"^/World/envs/env_\d+/", "/World/envs/env_0/", raw)


def set_exposure(cam: str, stops: float) -> bool:
    prim = stage.GetPrimAtPath(prim_of(cam))
    if not prim or not prim.IsValid():
        print(f"[cal] {cam}: prim 不存在 ({prim_of(cam)})")
        return False
    cam = UsdGeom.Camera(prim)
    attr = cam.GetExposureAttr()
    attr.Set(float(stops))
    return True


def means_and_save(tag: str) -> dict:
    for _ in range(14):
        env.sim.render()
        env.scene.update(env.physics_dt)
    from PIL import Image

    out = {}
    for cam in CAMS:
        arr = env.scene.sensors[cam].data.output["rgb"][0]
        arr = arr.torch if hasattr(arr, "torch") else arr
        arr = arr.detach().cpu().numpy()[..., :3]
        out[cam] = float(arr.mean())
        Image.fromarray(arr.astype(np.uint8)).save(os.path.join(args_cli.out, f"{tag}_{cam}.png"))
    print(f"[cal] {tag}: " + ", ".join(f"{c}={out[c]:6.1f}(目标{TARGET[c]:.1f})" for c in CAMS))
    return out


base = means_and_save("曝光0")
# 需要的 stops = log2(目标/当前)
needed = {c: math.log2(TARGET[c] / max(base[c], 0.1)) for c in CAMS}
print("[cal] 需要的曝光补偿(stops): " + ", ".join(f"{c}={needed[c]:+.2f}" for c in CAMS))

for cam in CAMS:
    set_exposure(cam, needed[cam])
after = means_and_save("已补偿")
print("[cal] 补偿后误差: " + ", ".join(f"{c}={after[c] - TARGET[c]:+.1f}" for c in CAMS))

env.close()
simulation_app.close()
