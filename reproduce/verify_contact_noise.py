#!/usr/bin/env python
"""检查底盘碰撞体的**接触力噪声**。

背景：底盘碰撞体是 kinematic 刚体，和静态几何（地板/台面）或自身机械臂深穿透时，
PhysX 会报出非物理的巨大接触力（实测有 1e4~1e11 N）。虚拟保险杠阈值只有 20 N，
这种噪声会让底盘莫名其妙进入 "⛔ 已挡住" 状态 → 该走的时候走不动。

本脚本在**完全静止**（不发任何速度指令）时采样 N 步接触力，报告：
  - 力的大小分布（最大/均值）与超过保险杠阈值的步数占比
  - 力的方向（判断是"地板顶上来"还是"手臂压下来"）
  - 保险杠是否被误触发

用法: reproduce/verify_contact_noise.py [--steps 300]
"""
import argparse
import dataclasses

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=300, help="静止采样步数（30 步 ≈ 1 秒仿真时间）")
parser.add_argument("--threshold", type=float, default=20.0, help="虚拟保险杠阈值（N）")
args = parser.parse_args()

app_launcher = AppLauncher(headless=True, enable_cameras=False)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.chassis import ChassisController  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402


def _is_image_term(term):
    func = getattr(term, "func", None)
    if func is not None and "image" in str(func):
        return True
    params = getattr(term, "params", None)
    return isinstance(params, dict) and "data_type" in params


def _strip(cfg_obj):
    if cfg_obj is None or not dataclasses.is_dataclass(cfg_obj):
        return
    for f in dataclasses.fields(cfg_obj):
        try:
            val = getattr(cfg_obj, f.name)
        except Exception:  # noqa: BLE001
            continue
        if val is None:
            continue
        if _is_image_term(val):
            setattr(cfg_obj, f.name, None)
        elif dataclasses.is_dataclass(val):
            _strip(val)


TASK = "LeIsaac-LeRobot-Kitchen-v0"
env_cfg = parse_env_cfg(TASK, device="cuda:0", num_envs=1)
for attr in ("observations", "events"):
    if hasattr(env_cfg, attr):
        _strip(getattr(env_cfg, attr))
for name in list(vars(env_cfg.scene).keys()):
    if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
        setattr(env_cfg.scene, name, None)
env_cfg.use_teleop_device(get_task_type(TASK))
env_cfg.recorders = None
for _n in ("time_out", "success"):
    if hasattr(env_cfg.terminations, _n):
        setattr(env_cfg.terminations, _n, None)

from leisaac.tasks.lerobot_kitchen.lerobot_kitchen_env_cfg import (  # noqa: E402
    BODY_COLLIDER_OFFSET,
    BODY_COLLIDER_SIZE,
)

env = gym.make(TASK, cfg=env_cfg).unwrapped
env.reset()

chassis = ChassisController(env, collision_force_threshold=args.threshold, lidar_stop_distance=0.0)
for _ in range(120):  # 静置到稳定
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
    chassis.update_sensors()

print("=" * 70)
print(f"底盘碰撞体: size={BODY_COLLIDER_SIZE} offset={BODY_COLLIDER_OFFSET}")
print(f"静止采样 {args.steps} 步，保险杠阈值 {args.threshold} N")
print("-" * 70)

sensor = env.scene.sensors["body_contact"]
forces, blocked_n = [], 0
for i in range(args.steps):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
    chassis.update_sensors()
    f = sensor.data.net_forces_w
    f = f.torch if hasattr(f, "torch") else f
    v = f[0].reshape(-1, 3).sum(dim=0).detach().cpu().numpy()
    forces.append(v)
    if chassis.last_contact_force > args.threshold:
        blocked_n += 1
    if (i + 1) % max(args.steps // 6, 1) == 0:
        print(f"  步 {i + 1:4d}: |F|={np.linalg.norm(v):12.2f} N  分量=({v[0]:+.1f}, {v[1]:+.1f}, {v[2]:+.1f})")

forces = np.array(forces)
mag = np.linalg.norm(forces, axis=1)
over = mag > args.threshold
print("-" * 70)
print(f"|F| 最大 {mag.max():.2f} N  均值 {mag.mean():.2f} N  中位 {np.median(mag):.2f} N")
print(f"超过阈值 {args.threshold} N 的步数: {int(over.sum())}/{len(mag)} ({over.mean() * 100:.1f}%)")
if over.any():
    fm = forces[over].mean(axis=0)
    nm = np.linalg.norm(fm)
    if nm > 1e-9:
        print(f"超阈值时的平均力方向: ({fm[0] / nm:+.2f}, {fm[1] / nm:+.2f}, {fm[2] / nm:+.2f})  —— "
              f"+z 为主=被地板/台面顶住；水平为主=和场景物体或自身手臂挤住")
print(f"被误判为 '⛔ 已挡住' 的步数: {blocked_n}/{args.steps}")
verdict = "✅ 接触力干净（静止时不应该有接触）" if over.mean() < 0.05 else "❌ 接触力噪声大，虚拟保险杠会被误触发"
print(f"结论: {verdict}")
print("=" * 70)

env.close()
simulation_app.close()
