#!/usr/bin/env python
"""验收：HDF5 → LeRobot v3 转换的**动作空间契约**（12 维主手 / 16 维键盘都必须对）。

为什么单独有这个脚本
-------------------
`scripts/convert/hdf5_to_lerobot_v3.py` 支持两类动作空间（判据与上游
`leisaac/utils/robot_utils.py::build_feature_from_env()` 完全一致）：

| 录制设备 | action 维度 | feature 名 | 数值 |
|---|---|---|---|
| `bi-so101leader`（真主手） | 12 | `left_*/right_*.pos` | 弧度 → 电机量值 |
| `bi-keyboard`（双臂键盘，IK） | 16 | `dim_0 … dim_15` | **原样存，不转换** |

`observation.state` 两类都必须是 12 维关节角→电机量值。
这份脚本把上面每一条都实测一遍 —— 通过就说明数据能直接进 LeRobot 训练。

用法（在装了 lerobot 的 `leisaac-lerobot-sim` 环境里跑，**不需要 isaaclab / 不需要起仿真**）::

    LR=<lerobot 环境的 python，例如 conda env leisaac-lerobot-sim 的 bin/python>
    $LR reproduce/verify_convert_action_space.py

    # 用自己的数据覆盖默认输入
    $LR reproduce/verify_convert_action_space.py \
        --hdf5-12 datasets/leader.hdf5 --hdf5-16 datasets/bikeyboard.hdf5

默认输入是两份合成数据（`reproduce/record_synthetic_episode.py` 录的，走真实录制路径）：
    datasets/synthetic_check_v2.hdf5     12 维（默认设备 bi-so101leader）
    datasets/synthetic_bi_keyboard.hdf5  16 维（--teleop_device bi-keyboard）
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CONVERT = os.path.join("scripts", "convert", "hdf5_to_lerobot_v3.py")

# ── 与 leisaac/assets/robots/lerobot.py 一致（独立实现一份，用来做"历史公式"对照）──────────
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
JOINT_LIMITS = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-160.0, 160.0),
    "gripper": (-10.0, 100.0),
}
MOTOR_LIMITS = {k: (-100.0, 100.0) for k in JOINT_ORDER}
MOTOR_LIMITS["gripper"] = (0.0, 100.0)

FEATURE_JOINT_NAMES = [
    "left_shoulder_pan.pos",
    "left_shoulder_lift.pos",
    "left_elbow_flex.pos",
    "left_wrist_flex.pos",
    "left_wrist_roll.pos",
    "left_gripper.pos",
    "right_shoulder_pan.pos",
    "right_shoulder_lift.pos",
    "right_elbow_flex.pos",
    "right_wrist_flex.pos",
    "right_wrist_roll.pos",
    "right_gripper.pos",
]

_failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  {'✅' if ok else '❌'} {name}" + (f"  （{detail}）" if detail else ""))
    if not ok:
        _failures.append(name)
    return ok


def legacy_radians_to_motor(a: np.ndarray) -> np.ndarray:
    """转换脚本用的"弧度 → 电机量值"映射（每 6 维一段），此处独立复刻一遍。"""
    a = np.asarray(a, dtype=np.float64)
    out = np.zeros_like(a)
    n = a.shape[1]
    for off in range(0, n, 6):
        for i, jn in enumerate(JOINT_ORDER):
            j = off + i
            if j >= n:
                break
            lo, hi = JOINT_LIMITS[jn]
            mlo, mhi = MOTOR_LIMITS[jn]
            out[:, j] = (a[:, j] - np.radians(lo)) / np.radians(hi - lo) * (mhi - mlo) + mlo
    return out


def read_parquet_stream(root: str, keys: tuple[str, ...]) -> dict[str, np.ndarray]:
    import pandas as pd

    files = sorted(glob.glob(os.path.join(root, "data", "**", "*.parquet"), recursive=True))
    if not files:
        raise RuntimeError(f"{root}/data 下没有 parquet")
    df = pd.concat([pd.read_parquet(p) for p in files], ignore_index=True)
    return {k: np.stack(df[k].to_numpy()) for k in keys}


def convert(hdf5: str, repo_id: str, root: str, skip: int) -> str:
    out_dir = os.path.join(root, repo_id)
    cmd = [
        sys.executable,
        CONVERT,
        "--hdf5", hdf5,
        "--repo_id", repo_id,
        "--root", root,
        "--task", f"verify {repo_id}",
        "--skip_first_frames", str(skip),
        "--video_backend", "pyav",
    ]
    print(f"\n[verify] $ {' '.join(cmd)}")
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True)
    for line in proc.stdout.splitlines():
        if line.startswith("[convert]"):
            print("   " + line)
    if proc.returncode != 0:
        print(proc.stdout[-2000:])
        print(proc.stderr[-2000:])
        raise RuntimeError(f"转换失败（退出码 {proc.returncode}）")
    return out_dir


def main() -> int:
    ap = argparse.ArgumentParser(description="验收 HDF5→LeRobot v3 的动作空间契约")
    ap.add_argument("--hdf5-12", default="datasets/synthetic_check_v2.hdf5", help="12 维（真主手）HDF5")
    ap.add_argument("--hdf5-16", default="datasets/synthetic_bi_keyboard.hdf5", help="16 维（双臂键盘）HDF5")
    ap.add_argument("--root", default=None, help="转换输出根目录（默认临时目录，跑完删）")
    ap.add_argument("--skip_first_frames", type=int, default=5)
    args = ap.parse_args()

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.datasets.utils import dataset_to_policy_features
    except ImportError as exc:
        print(f"[verify] ❌ 需要 lerobot：{exc}\n  请用 leisaac-lerobot-sim 环境运行（见操作指南 §7）")
        return 2

    import h5py

    for p in (args.hdf5_12, args.hdf5_16):
        if not os.path.isfile(os.path.join(REPO, p)) and not os.path.isfile(p):
            print(f"[verify] ❌ 找不到 {p}\n"
                  f"  先录一份：见 reproduce/record_synthetic_episode.py 的文档头")
            return 2

    tmp = None
    root = args.root
    if root is None:
        tmp = tempfile.mkdtemp(prefix="lerobot_verify_")
        root = tmp
    root = os.path.abspath(root)
    os.makedirs(root, exist_ok=True)
    print(f"[verify] 转换输出根目录：{root}")

    try:
        # ── ① 12 维（真主手）：关节命名 + 弧度→电机量值 ─────────────────────────────
        print("\n" + "=" * 64)
        print("① 12 维动作（真主手 bi-so101leader）")
        print("=" * 64)
        d12 = convert(args.hdf5_12, "verify_leader_12", root, args.skip_first_frames)
        info12 = json.load(open(os.path.join(d12, "meta", "info.json")))
        arr12 = read_parquet_stream(d12, ("action", "observation.state"))
        check("action feature = 12 维", list(info12["features"]["action"]["shape"]) == [12],
              str(info12["features"]["action"]["shape"]))
        check("action 用真关节名", info12["features"]["action"]["names"] == FEATURE_JOINT_NAMES)
        check("observation.state = 12 维", list(info12["features"]["observation.state"]["shape"]) == [12])
        with h5py.File(os.path.join(REPO, args.hdf5_12), "r") as f:
            demos = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
            raw = np.concatenate([f[f"data/{n}/actions"][args.skip_first_frames:] for n in demos], axis=0)
            lp = np.concatenate([f[f"data/{n}/obs/left_joint_pos"][args.skip_first_frames:] for n in demos], axis=0)
            rp = np.concatenate([f[f"data/{n}/obs/right_joint_pos"][args.skip_first_frames:] for n in demos], axis=0)
        exp_a = legacy_radians_to_motor(raw)
        exp_s = np.concatenate([legacy_radians_to_motor(lp), legacy_radians_to_motor(rp)], axis=1)
        check("action 数值 == 弧度→电机量值（与改动前逐位一致）",
              np.allclose(arr12["action"], exp_a, atol=1e-5),
              f"max|Δ|={float(np.abs(arr12['action'] - exp_a).max()):.2e}")
        check("state 数值 == 左右关节角→电机量值",
              np.allclose(arr12["observation.state"], exp_s, atol=1e-5),
              f"max|Δ|={float(np.abs(arr12['observation.state'] - exp_s).max()):.2e}")

        # ── ② 16 维（双臂键盘）：dim_i 命名 + 原样存 ────────────────────────────────
        print("\n" + "=" * 64)
        print("② 16 维动作（双臂键盘 bi-keyboard，IK 位姿增量）")
        print("=" * 64)
        d16 = convert(args.hdf5_16, "verify_bikeyboard_16", root, args.skip_first_frames)
        info16 = json.load(open(os.path.join(d16, "meta", "info.json")))
        arr16 = read_parquet_stream(d16, ("action", "observation.state"))
        check("action feature = 16 维", list(info16["features"]["action"]["shape"]) == [16],
              str(info16["features"]["action"]["shape"]))
        check("action 用 dim_0..dim_15 命名（不是关节名，避免误导）",
              info16["features"]["action"]["names"] == [f"dim_{i}" for i in range(16)])
        check("observation.state 仍是 12 维（与录制设备无关）",
              list(info16["features"]["observation.state"]["shape"]) == [12])
        with h5py.File(os.path.join(REPO, args.hdf5_16), "r") as f:
            demos = sorted(f["data"].keys(), key=lambda s: int(s.split("_")[1]))
            raw16 = np.concatenate([f[f"data/{n}/actions"][args.skip_first_frames:] for n in demos], axis=0)
            lp16 = np.concatenate([f[f"data/{n}/obs/left_joint_pos"][args.skip_first_frames:] for n in demos], axis=0)
            rp16 = np.concatenate([f[f"data/{n}/obs/right_joint_pos"][args.skip_first_frames:] for n in demos], axis=0)
        check("action 数值 == HDF5 原值（**不做**弧度→电机量值转换）",
              np.allclose(arr16["action"], raw16, atol=1e-6),
              f"max|Δ|={float(np.abs(arr16['action'] - raw16).max()):.2e}")
        exp_s16 = np.concatenate([legacy_radians_to_motor(lp16), legacy_radians_to_motor(rp16)], axis=1)
        check("state 数值 == 左右关节角→电机量值（与 12 维数据同构）",
              np.allclose(arr16["observation.state"], exp_s16, atol=1e-5))

        # ── ③ LeRobot 侧能识别成 policy 特征 ───────────────────────────────────────
        print("\n" + "=" * 64)
        print("③ LeRobot 把它识别成可训练的 policy 特征")
        print("=" * 64)
        ds16 = LeRobotDataset(repo_id="verify_bikeyboard_16", root=d16, video_backend="pyav")
        pf = dataset_to_policy_features(ds16.features)
        check("dataset 能加载", ds16.num_frames > 0, f"episodes={ds16.num_episodes} frames={ds16.num_frames}")
        check("action → FeatureType.ACTION (16,)", tuple(pf["action"].shape) == (16,),
              str(pf["action"].type))
        check("observation.state → FeatureType.STATE (12,)", tuple(pf["observation.state"].shape) == (12,))
        n_visual = sum(1 for f in pf.values() if str(f.type).endswith("VISUAL"))
        check("三路相机都在特征里", n_visual == 3, f"visual={n_visual}")
        import torch
        from torch.utils.data import DataLoader

        batch = next(iter(DataLoader(ds16, batch_size=2, shuffle=False)))
        check("DataLoader 能拼 batch",
              tuple(batch["action"].shape) == (2, 16) and tuple(batch["observation.state"].shape) == (2, 12),
              f"action={tuple(batch['action'].shape)} state={tuple(batch['observation.state'].shape)}")
        check("batch 里没有 NaN/Inf", bool(torch.isfinite(batch["action"]).all()))
    finally:
        if tmp is not None:
            import shutil

            shutil.rmtree(tmp, ignore_errors=True)

    print("\n" + "=" * 64)
    if _failures:
        print("❌ 未通过：" + "、".join(_failures))
    else:
        print("✅ 动作空间契约全部通过（12 维主手 / 16 维键盘都能进 LeRobot 训练）")
    print("=" * 64)
    print("下一步（真正跑一遍训练，3 步 smoke）：")
    print("  <leisaac-lerobot-sim>/bin/lerobot-train \\")
    print("      --dataset.repo_id=<数据集名> --dataset.root=<数据集目录> \\")
    print("      --dataset.video_backend=pyav --policy.type=act --policy.device=cuda \\")
    print("      --output_dir=/tmp/act_smoke --wandb.enable=false --policy.push_to_hub=false \\")
    print("      --steps=3 --batch_size=2")
    return 1 if _failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
