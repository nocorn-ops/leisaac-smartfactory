#!/usr/bin/env python
"""抓取厨房场景"出生态"与"静置后"的相机画面 + 物体位姿，用来判断盘子/橙子到底是不是倒置。

用法（仓库根目录）:
    reproduce/capture_startup.py --out /tmp/startup            # 默认：前视+双腕相机
    reproduce/capture_startup.py --out /tmp/startup --settle 150
输出: <out>/{spawn,settled}/front.png 等 + 控制台位姿表
"""
import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--out", type=str, default="/tmp/startup")
parser.add_argument("--settle", type=int, default=150, help="静置步数（与 teleop 的 --settle_steps 一致）")
parser.add_argument("--extra_settle", type=int, default=0, help="静置后再额外步进（看是否继续变）")
parser.add_argument("--render", action="store_true", help="静置时带渲染（render=True）；不带渲染时相机输出会是全白图")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402

OBJS = ["Plate", "Orange002", "Orange003"]


def _t(x):
    x = x.torch if hasattr(x, "torch") else x
    return x.detach()


env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.use_teleop_device(get_task_type(args_cli.task))
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()


def report(tag: str) -> None:
    for n in OBJS:
        obj = env.scene[n]
        pos = _t(obj.data.root_pos_w)[0]
        quat = _t(obj.data.root_quat_w)[0]
        print(f"[pose] {tag:8s} {n:10s} pos=({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f}) quat=({quat[0]:.4f},{quat[1]:.4f},{quat[2]:.4f},{quat[3]:.4f})")


def save_cameras(out_dir: str) -> None:
    import numpy as np
    from PIL import Image

    os.makedirs(out_dir, exist_ok=True)
    for name, sensor in env.scene.sensors.items():
        out = getattr(sensor.data, "output", None)
        rgb = out.get("rgb") if isinstance(out, dict) else None
        if rgb is None:
            continue
        arr = rgb[0].detach().cpu().numpy()
        if arr.dtype != np.uint8:
            arr = (arr[..., :3].clip(0.0, 1.0) * 255.0).astype(np.uint8)
        path = os.path.join(out_dir, f"{name}.png")
        Image.fromarray(arr[..., :3]).save(path)
        print(f"[cam] {name}: mean={float(arr.mean()):.1f} -> {path}")


report("spawn")
save_cameras(os.path.join(args_cli.out, "spawn"))

for _ in range(args_cli.settle):
    env.sim.step(render=bool(args_cli.render))
    env.scene.update(env.physics_dt)
report("settled")
save_cameras(os.path.join(args_cli.out, "settled"))

for _ in range(args_cli.extra_settle):
    env.sim.step(render=bool(args_cli.render))
    env.scene.update(env.physics_dt)
if args_cli.extra_settle:
    report("extra")
    save_cameras(os.path.join(args_cli.out, "extra"))

env.close()
simulation_app.close()
