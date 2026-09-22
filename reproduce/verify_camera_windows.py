#!/usr/bin/env python
"""验证"相机视口窗口"：建窗口 → 检查数量/绑定 → 渲染 → 把每个窗口的画面存 PNG。

用法（需要 GUI，窗口会自动退出）:
    reproduce/verify_camera_windows.py --out /tmp/camwin --visualizer kit
"""
import argparse
import dataclasses
import os
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--out", type=str, default="/tmp/camwin")
parser.add_argument("--seconds", type=float, default=12.0)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402

from leisaac.utils.camera_view import create_camera_view_windows  # noqa: E402

env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.use_teleop_device(get_task_type(args_cli.task))
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()
for _ in range(60):
    env.sim.step(render=True)
    env.scene.update(env.physics_dt)

import omni.kit.viewport.utility as vp_utils  # noqa: E402

print(f"[verify] 建窗口前 viewport 数量 = {vp_utils.get_num_viewports()}")
windows = create_camera_view_windows(env)
print(f"[verify] 建窗口后 viewport 数量 = {vp_utils.get_num_viewports()}（期望 +{len(windows)}）")

names = ["left_wrist", "right_wrist", "front"]
for name in names:
    win_name = f"camera: {name}"
    vp = vp_utils.get_viewport_from_window_name(win_name)
    cam = vp_utils.get_viewport_window_camera_string(win_name) if vp is not None else None
    print(f"[verify] 窗口 '{win_name}': viewport={'有' if vp is not None else '无'} 相机={cam}")

# 让窗口渲染几帧
t0 = time.time()
while time.time() - t0 < 5.0:
    env.sim.render()
    time.sleep(0.02)

os.makedirs(args_cli.out, exist_ok=True)
for name in names:
    vp = vp_utils.get_viewport_from_window_name(f"camera: {name}")
    if vp is None:
        continue
    path = os.path.join(args_cli.out, f"window_{name}.png")
    vp_utils.capture_viewport_to_file(vp, path)
    print(f"[verify] 抓取窗口画面 -> {path}")

t0 = time.time()
while time.time() - t0 < args_cli.seconds:
    env.sim.render()
    time.sleep(0.02)

env.close()
simulation_app.close()
