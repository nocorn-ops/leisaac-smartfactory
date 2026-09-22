#!/usr/bin/env python
"""对比"相机视口窗口"和"相机图像面板"两种显示方式的开销，并检查面板收到的图像数据是否正常。

用法: reproduce/bench_camera_view.py --steps 60 --visualizer kit
"""
import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--steps", type=int, default=60)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.camera_view import CameraImagePanel, create_camera_view_windows  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402

env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.use_teleop_device(get_task_type(args_cli.task))
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()
for _ in range(60):
    env.sim.step(render=True)
    env.scene.update(env.physics_dt)

zero = torch.zeros(env.action_space.shape[-1], device=env.device).unsqueeze(0)


def bench(tag: str, hook=None) -> float:
    for _ in range(5):
        env.step(zero)
        if hook:
            hook()
    t0 = time.time()
    for _ in range(args_cli.steps):
        env.step(zero)
        if hook:
            hook()
    dt = (time.time() - t0) / args_cli.steps * 1000.0
    print(f"[bench] {tag}: {dt:.1f} ms/步 (≈{1000.0 / dt:.1f} Hz)")
    return dt


a = bench("基线（不显示相机）")

panel = CameraImagePanel(env)
panel.update()
for name, provider in panel._providers.items():
    sensor = panel._sensors[name]
    rgb = sensor.data.output["rgb"]
    arr = rgb[0]
    arr = arr.torch if hasattr(arr, "torch") else arr
    arr = arr.detach().cpu().numpy()
    print(f"[bench] 面板 {name}: shape={arr.shape} dtype={arr.dtype} mean={float(arr.mean()):.1f}")

b = bench(f"图像面板（{panel.names}，每 3 帧刷一次）", hook=panel.update)

windows = create_camera_view_windows(env)
c = bench(f"视口窗口（{len(windows)} 个）")

print(f"[bench] 面板开销 +{b - a:.1f} ms/步；视口窗口开销 +{c - a:.1f} ms/步")

env.close()
simulation_app.close()
