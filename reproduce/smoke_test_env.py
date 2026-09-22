#!/usr/bin/env python
"""LeIsaac 环境冒烟测试 —— 在 Isaac Sim 容器内验证任务能否构建并稳定跑若干步。

设计目标（验证"环境能否复现"的最小闭环）：
  1. 解析任务配置（触发所有 USD 资源路径解析/检查，如厨房 scene、机械臂、躯干）
  2. gym.make + reset()，真正加载场景并启动 PhysX
  3. 零动作跑 N 步（PD 控制器保持关节位置），确认不崩溃

用法（Isaac Sim 6.0.1 的 python.sh, 仓库根目录为 cwd）:
  "${ISAACSIM_DIR}/python.sh" -u reproduce/smoke_test_env.py --task LeIsaac-SmartFactory-v0 --steps 60
  "${ISAACSIM_DIR}/python.sh" -u reproduce/smoke_test_env.py --task LeIsaac-SmartFactory-v0 --cameras --steps 30

说明:
  * 本脚本**固定 headless**（不依赖 X display）。
  * ⚠️ IsaacLab 3.0 起 `--headless` / `--enable_cameras` **不再是 CLI 参数**（改成 AppLauncher 的
    配置键）。所以这里用 `--cameras` 控制相机；写 `--enable_cameras` 会报 unrecognized arguments。
  * headless + 相机可能吃显存（12 GB 卡上紧张），默认会剔除 image 观测词条与对应相机资产以
    最小化显存占用 —— 传入 `--cameras` 保留。
  * 退出码 0 = 通过。
"""
import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)

import os
import sys

# --- 必须在 import isaaclab 之前解析参数 ---
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="LeIsaac 环境冒烟测试")
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0", help="任务名")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=60, help="零动作步数")
parser.add_argument(
    "--cameras",
    action="store_true",
    help="保留相机（3.0 下相机可用；默认剔除是为了跑得更轻，但带相机的域随机化任务需要它）",
)
AppLauncher.add_app_launcher_args(parser)  # IsaacLab 3.0: 不再提供 --headless/--enable_cameras CLI 参数
args_cli = parser.parse_args()

# --- IsaacLab 3.0 起 headless/enable_cameras 改为 AppLauncher 配置键（非 CLI 参数）---
# 冒烟测试固定 headless 模式（不依赖 X display）；相机默认关掉以省显存/时间，--cameras 可打开。
# 注意：AppLauncher 会修改传入的 namespace（消费掉部分键），故用独立变量保存标志。
ENABLE_CAMERAS = bool(args_cli.cameras)
args_cli.headless = True
args_cli.enable_cameras = ENABLE_CAMERAS

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import dataclasses
import time

import gymnasium as gym
import torch

import leisaac  # noqa: F401  注册任务
from isaaclab_tasks.utils import parse_env_cfg
from leisaac.utils.env_utils import get_task_type


def _is_image_term(term) -> bool:
    """判断一个 ObsTerm 是否为图像观测。

    图像词条的 ``func`` 为 ``mdp.image``（可能被包装），或其 ``params`` 含图像专用的 ``data_type``。
    """
    func = getattr(term, "func", None)
    if func is not None and (getattr(func, "__name__", None) == "image" or "image" in str(func)):
        return True
    params = getattr(term, "params", None)
    return isinstance(params, dict) and "data_type" in params


def _strip_image_terms(cfg_obj) -> int:
    """递归剔除观测配置中的 image 词条，返回剔除数量。IsaacLab 观测/事件/动作管理器
    均跳过值为 None 的词条。"""
    removed = 0
    if cfg_obj is None or not dataclasses.is_dataclass(cfg_obj):
        return 0
    for f in dataclasses.fields(cfg_obj):
        try:
            val = getattr(cfg_obj, f.name)
        except Exception:
            continue
        if val is None:
            continue
        # 词条对象（ObsTerm 本身也是 dataclass，须先判断再递归，否则会漏掉图像词条）
        if _is_image_term(val):
            setattr(cfg_obj, f.name, None)
            removed += 1
            continue
        if dataclasses.is_dataclass(val):  # 嵌套 group（如 policy 观测组）
            removed += _strip_image_terms(val)
    return removed


def main() -> int:
    print(f"[smoke] 任务: {args_cli.task}  num_envs={args_cli.num_envs}  steps={args_cli.steps}")

    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.recorders = None  # 冒烟测试不录制（避免 HDF5 文件锁等副作用）

    if not ENABLE_CAMERAS:
        removed = 0
        for attr in ("observations", "events"):
            if hasattr(env_cfg, attr):
                removed += _strip_image_terms(getattr(env_cfg, attr))
        # 同时剔除场景里的相机传感器，否则会因"未开 --enable_cameras 却 spawn 相机"而报错
        from isaaclab.sensors import TiledCameraCfg

        for name in list(vars(env_cfg.scene).keys()):
            if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
                setattr(env_cfg.scene, name, None)
                removed += 1
        print(f"[smoke] 已剔除 {removed} 个相机观测词条/传感器(--enable_cameras 可保留)")

    task_type = get_task_type(args_cli.task)
    env_cfg.use_teleop_device(task_type)
    env_cfg.recorders = None

    print("[smoke] 构建环境 ...")
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.reset()
    print("[smoke] 环境构建成功，开始零动作步进 ...")

    # 构造零动作
    action = None
    if hasattr(env, "action_manager") and hasattr(env.action_manager, "zero_action"):
        try:
            action = env.action_manager.zero_action()
        except Exception:
            action = None
    if action is None:
        # 维度必须跟动作空间一致：双臂任务 12 维、单臂任务 6 维（写死 12 会让单臂任务报
        # "Invalid action shape, expected: 6, received: 12"）
        try:
            num_actions = env.action_space.shape[-1]
        except Exception:  # noqa: BLE001
            num_actions = getattr(env.action_manager, "total_action_dim", None) or 12
        action = torch.zeros(env.num_envs, num_actions, device=env.device)
        print(f"[smoke] 零动作维度 = {num_actions}")

    t0 = time.time()
    for i in range(1, args_cli.steps + 1):
        env.step(action)
        if i % 20 == 0:
            print(f"[smoke] step {i}/{args_cli.steps} ...")
    dt = time.time() - t0

    print(f"[smoke] OK: {args_cli.task} 构建并稳定运行 {args_cli.steps} 步 ({dt:.1f}s)，环境复现通过。")
    env.close()
    return 0


if __name__ == "__main__":
    rc = 1
    try:
        rc = main()
    except Exception as e:  # noqa: BLE001  —— 冒烟测试要明确报出失败
        import traceback

        traceback.print_exc()
        print(f"[smoke] FAILED: {type(e).__name__}: {e}")
        rc = 1
    finally:
        simulation_app.close()
    sys.exit(rc)
