#!/usr/bin/env python
"""无主手验证"采数据"链路：跑合成 episode 并导出 HDF5。

走的是和 `scripts/environments/teleoperation/teleop_se3_agent.py --record` **完全相同**的
录制路径（StreamingRecorderManager + 同一套 recorders 配置），只是动作由程序生成
（小幅正弦），所以不需要主手硬件即可验证 HDF5 录制，并产出一个小的数据集
供后续"转换 → 推理"链路测试。

用法（仓库根目录）:
  env -u CONDA_PREFIX -u CONDA_DEFAULT_ENV -u CONDA_SHLVL -u CONDA_EXE -u VIRTUAL_ENV \
    OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y \
    LD_LIBRARY_PATH="$PWD/reproduce/compat_libs_isaac6" \
    <IsaacSim>/python.sh -u reproduce/record_synthetic_episode.py \
      --task LeIsaac-SmartFactory-v0 --output datasets/synthetic_check.hdf5 \
      --episodes 2 --steps 40 --enable_cameras
      # 产 16 维（双臂键盘）动作空间的数据再加：--teleop_device bi-keyboard
"""
import multiprocessing

if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)

import argparse
import math
import os
import sys
import time

# ── 让本仓库的 `leisaac` 包可被 import（不依赖 editable 安装 / 外部 PYTHONPATH）──
# 打包给赛队时仓库可能被解压到任意目录、也不一定跑过 `pip install -e source/leisaac`；
# 这里从本文件往上找到含 `source/leisaac` 的仓库根，自己补 sys.path（不依赖 CWD）。
_p = os.path.dirname(os.path.abspath(__file__))
while _p != os.path.dirname(_p):
    if os.path.isdir(os.path.join(_p, "source", "leisaac")):
        sys.path.insert(0, os.path.join(_p, "source", "leisaac"))
        break
    _p = os.path.dirname(_p)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="合成 episode 录制验证")
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--output", type=str, default="datasets/synthetic_check.hdf5")
parser.add_argument("--episodes", type=int, default=2, help="录制几个 episode")
parser.add_argument("--steps", type=int, default=40, help="每个 episode 步数")
parser.add_argument("--amplitude", type=float, default=0.2, help="动作正弦幅度")
parser.add_argument("--num_envs", type=int, default=1)
# 不传就用任务默认设备（双臂任务 → bi-so101leader，12 维动作）；
# 传 `bi-keyboard` 可产出 **16 维**动作的 HDF5（用来验证非关节空间数据的转换/训练链路）。
parser.add_argument(
    "--teleop_device",
    type=str,
    default=None,
    help="覆盖任务默认遥操设备（例：bi-keyboard → 16 维动作）。默认 None = 按任务推断",
)
# 相机开关：IsaacLab 3.0 里 --enable_cameras 是 AppLauncher 配置键、不能直接当 argparse 字段名，
# 用别名 dest 保留命令行写法，解析后写回。
parser.add_argument("--enable_cameras", dest="enable_cameras_flag", action="store_true", default=False)
parser.add_argument("--headless", dest="headless_flag", action="store_true", default=False)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = bool(args_cli.enable_cameras_flag)
args_cli.headless = True if args_cli.headless_flag else None

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab.managers import DatasetExportMode, TerminationTermCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

import leisaac  # noqa: E402, F401
from leisaac.enhance.managers import StreamingRecorderManager  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402
from leisaac.utils.sim_modes import ensure_observations  # noqa: E402


def manual_terminate(env, success: bool) -> None:
    """与遥操脚本同款：把 success 终止项临时设为恒定值，让录制器按"成功/失败"导出。"""
    if hasattr(env, "termination_manager"):
        env.termination_manager.set_term_cfg(
            "success",
            TerminationTermCfg(
                func=lambda e: torch.full(
                    (e.num_envs,), success, dtype=torch.bool, device=e.device
                )
            ),
        )
        env.termination_manager.compute()


def main() -> int:
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    device = args_cli.teleop_device or get_task_type(args_cli.task)
    env_cfg.use_teleop_device(device)

    # 场地任务（SmartFactory）的观测是空的，录制需要观测组 → 和统一入口一样按双臂模板注入。
    # 已有观测组的任务（厨房等）返回 False，不改动。
    if ensure_observations(env_cfg):
        print("[rec] 任务没有观测组 → 已按双臂模板注入（左右关节 + 三路相机）")
    print(f"[rec] task={args_cli.task} teleop_device={device}")

    out_path = os.path.abspath(args_cli.output)
    out_dir = os.path.dirname(out_path)
    out_stem = os.path.splitext(os.path.basename(out_path))[0]
    if os.path.exists(out_path):
        print(f"[rec] 目标文件已存在，停止（不覆盖）：{out_path}")
        return 2

    # —— 与 teleop_se3_agent.py --record 相同的录制器配置 ——
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_ALL
    env_cfg.recorders.dataset_export_dir_path = out_dir
    env_cfg.recorders.dataset_filename = out_stem
    if not hasattr(env_cfg.terminations, "success"):
        setattr(env_cfg.terminations, "success", None)
    env_cfg.terminations.success = TerminationTermCfg(
        func=lambda env: torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    )

    print(f"[rec] 构建环境 {args_cli.task} ...", flush=True)
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    del env.recorder_manager
    env.recorder_manager = StreamingRecorderManager(env_cfg.recorders, env)
    env.recorder_manager.flush_steps = 100
    env.recorder_manager.compression = "lzf"

    action_dim = env.action_space.shape[-1]
    print(f"[rec] 动作维度 = {action_dim}，开始录制 {args_cli.episodes} 个 episode × {args_cli.steps} 步", flush=True)

    env.reset()
    for ep in range(args_cli.episodes):
        for i in range(args_cli.steps):
            phase = 2.0 * math.pi * i / max(args_cli.steps - 1, 1)
            action = torch.full(
                (env.num_envs, action_dim),
                math.sin(phase) * args_cli.amplitude,
                dtype=torch.float32,
                device=env.device,
            )
            env.step(action)
        # 标记成功 + reset ⇒ 录制器导出这一条（和遥操按 N 再按 R 的效果一致）
        manual_terminate(env, True)
        env.reset()
        # 关键：遥操脚本在 reset 后会 manual_terminate(env, False) 把成功标记关掉。
        # 不关掉的话，success 终止项会一直为真，后续每一步都触发终止→reset→再导出，
        # 结果就是一堆只有 1 帧的假 episode。
        manual_terminate(env, False)
        print(f"[rec] episode {ep + 1}/{args_cli.episodes} 已导出", flush=True)

    env.close()
    size = os.path.getsize(out_path) if os.path.isfile(out_path) else 0
    print(f"[rec] 完成：{out_path} ({size / 1e6:.2f} MB)")
    if size == 0:
        print("[rec] FAILED: 没有生成 HDF5")
        return 1

    # 回读校验（h5py 在 6.0.1 的 python 里可用）
    rc = 0
    try:
        import h5py

        with h5py.File(out_path, "r") as f:
            demos = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
            print(f"[rec] 校验：episodes={len(demos)} {demos[:5]}")
            if demos:
                d = f[f"data/{demos[0]}"]
                print(f"[rec] 校验：第一条的 keys={sorted(d.keys())}")
                if "obs" in d:
                    for k in sorted(d["obs"].keys()):
                        print(f"[rec]   obs/{k}: shape={d[f'obs/{k}'].shape} dtype={d[f'obs/{k}'].dtype}")
                # 每条都应是完整长度（否则说明 success 标记没清掉，产生了 1 帧的假 episode）
                lens = [f[f"data/{n}/actions"].shape[0] for n in demos]
                print(f"[rec] 校验：每条帧数={lens}")
                if len(demos) != args_cli.episodes or any(n != args_cli.steps for n in lens):
                    print(f"[rec] 警告：期望 {args_cli.episodes} 条 × {args_cli.steps} 帧，实际不符")
                    rc = 1
    except Exception as e:  # noqa: BLE001
        print(f"[rec] 回读校验失败（不影响录制）：{type(e).__name__}: {e}")
    return rc


if __name__ == "__main__":
    t0 = time.time()
    rc = 1
    try:
        rc = main()
    except Exception:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        rc = 1
    finally:
        print(f"[rec] 退出码={rc} 用时={time.time() - t0:.1f}s", flush=True)
        simulation_app.close()
    raise SystemExit(rc)
