#!/usr/bin/env python
"""HDF5（IsaacLab/LeIsaac 录制） → LeRobot v3 数据集。**只依赖 h5py + lerobot**，不需要 isaaclab、不需要起仿真。

为什么要单独写一个：仓库原有的 `scripts/convert/isaaclab2lerobotv3.py` 需要**同一个解释器里同时有
lerobot 和 isaaclab**（它 `gym.make()` 建环境，只为了调 `env.cfg.build_lerobot_frame()` 和
`build_feature_from_env()`）。但那两件事本质是"改名 + 搬运 + 拼 feature 表"，纯数据操作 ——
这里用 h5py 直接读 HDF5 复刻同样的映射，于是可以在**本项目的专用 lerobot 环境**（lerobot 0.4.2）
里跑，不用去凑一个混合解释器。

复刻自（数值/映射必须一致）：
- `leisaac/tasks/template/bi_arm_env_cfg.py::build_lerobot_frame`
  （action = `actions[t]` 经 `convert_leisaac_action_to_lerobot`；state = 左右 `joint_pos` 各转一次后拼接；
   图像 = `obs/<cam>[t]`；跳过每集前 5 帧）
- `leisaac/utils/robot_utils.py::build_feature_from_env`
  （**动作维度 ≠ 关节数时不转换、也不按关节命名**，见下；相机 feature 形状 **[H, W, C]**，
   lerobot 会在 policy 侧转成 [C, H, W]，与现有 checkpoint 的 `[3,240,320]` 一致）
- `leisaac/assets/robots/lerobot.py` 的两个限位表（下面内联复制，避免 import isaaclab）

**两类动作空间（与上游 `build_feature_from_env()` 的判据完全一致）**：

| 录制设备 | `actions` 维度 | action feature | 数值转换 |
|---|---|---|---|
| `bi-so101leader` / `so101leader`（真主手） | 12 | 12 个 `left_*/right_*.pos` 名字 | 弧度 → 电机量值 |
| `bi-keyboard`（双臂键盘，IK） | **16** | `dim_0 … dim_15` | **原样存，不转换** |
| `so101_state_machine` 等 IK 设备 | 6+1 / 2×(6+1) | `dim_i` | 原样存，不转换 |

原因：键盘/状态机走的是"末端位姿增量 + IK"，16 维里前 6 维是笛卡尔增量（米/弧度），
**不是关节角**，套"弧度→电机量值"映射只会得到没有物理意义的数。上游的判据是

    action_dim != len(default_feature_joint_names)  →  action_align = False

`observation.state` **永远**是 12 维关节角→电机量值（来自 `obs/left_joint_pos` /
`right_joint_pos`），和录制设备无关 —— LeRobot 允许 action 与 state 维度不同，
所以 16 维动作 + 12 维状态是合法的、可直接训练的。

用法（在 leisaac-lerobot-sim 环境里跑）::

    <装了 lerobot 的 python> scripts/convert/hdf5_to_lerobot_v3.py \
        --hdf5 datasets/kitchen_biarm.hdf5 \
        --repo_id kitchen_biarm_v1_local \
        --root datasets/lerobot \
        --task "Pick oranges with the leRobot bi-arm manipulator."
"""
from __future__ import annotations

import argparse
import os
import re
import sys

import h5py
import numpy as np

# ── 与 leisaac/assets/robots/lerobot.py 保持一致（该模块 import isaaclab，这里不引入）──────────
SO101_FOLLOWER_USD_JOINT_LIMLITS = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 90.0),
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-160.0, 160.0),
    "gripper": (-10.0, 100.0),
}
SO101_FOLLOWER_MOTOR_LIMITS = {
    "shoulder_pan": (-100.0, 100.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-100.0, 100.0),
    "wrist_flex": (-100.0, 100.0),
    "wrist_roll": (-100.0, 100.0),
    "gripper": (0.0, 100.0),
}
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
#: 与本项目 env_cfg.default_feature_joint_names 一致
DEFAULT_FEATURE_JOINT_NAMES = [
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
CAMERAS = ("left_wrist", "right_wrist", "front")
#: 关节空间的维度（= len(DEFAULT_FEATURE_JOINT_NAMES)）。动作维度等于它才做数值转换。
STATE_DIM = len(DEFAULT_FEATURE_JOINT_NAMES)


def convert_leisaac_action_to_lerobot(action: np.ndarray) -> np.ndarray:
    """弧度 → 电机值（与 `leisaac.utils.robot_utils.convert_leisaac_action_to_lerobot` 等价）。

    支持单臂(6)/双臂(12)：每 6 维一段，按左右顺序各转一次。
    """
    action = np.asarray(action, dtype=np.float64)
    if action.ndim == 1:
        action = action[None, :]
    out = np.zeros_like(action)
    num_joints = action.shape[1]
    for offset in range(0, num_joints, 6):
        for idx, joint_name in enumerate(JOINT_ORDER):
            j = offset + idx
            if j >= num_joints:
                break
            j_lo, j_hi = SO101_FOLLOWER_USD_JOINT_LIMLITS[joint_name]
            m_lo, m_hi = SO101_FOLLOWER_MOTOR_LIMITS[joint_name]
            deg = action[:, j] - np.radians(j_lo)
            out[:, j] = deg / np.radians(j_hi - j_lo) * (m_hi - m_lo) + m_lo
    return out


def build_features(
    image_shape: tuple[int, int, int],
    action_dim: int,
    action_names: list[str],
    use_videos: bool,
    fps: int,
) -> dict:
    """复刻 `build_feature_from_env` 的 feature 表（相机形状用 [H, W, C]）。

    与上游一致：`action` 用**传入的** `action_names`（关节空间用真名字，非关节空间用 `dim_i`），
    `observation.state` 永远用 12 个关节名。
    """
    h, w, c = image_shape
    features = {
        "action": {"dtype": "float32", "shape": (action_dim,), "names": action_names},
        "observation.state": {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": DEFAULT_FEATURE_JOINT_NAMES,
        },
    }
    for cam in CAMERAS:
        item = {
            "dtype": "video" if use_videos else "image",
            "shape": (h, w, c),
            "names": ["height", "width", "channels"],
        }
        if use_videos:
            item["video_info"] = {
                "video.height": h,
                "video.width": w,
                "video.codec": "av1",
                "video.pix_fmt": "yuv420p",
                "video.is_depth_map": False,
                "video.fps": float(fps),
                "video.channels": c,
                "has_audio": False,
            }
        features[f"observation.images.{cam}"] = item
    return features


def list_episodes(f, selector: str | None) -> list[str]:
    """按数字序返回 demo 名字；selector 支持 'all' / '0,2,5' / '0-3'。"""
    keys = [k for k in f["data"].keys() if k.startswith("demo_")]
    keys.sort(key=lambda k: int(re.sub(r"\D", "", k) or 0))
    if selector is None or selector.strip().lower() in ("", "all"):
        return keys
    want: list[int] = []
    for part in selector.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            want.extend(range(int(a), int(b) + 1))
        else:
            want.append(int(part))
    picked = []
    for i in want:
        name = f"demo_{i}"
        if name in keys:
            picked.append(name)
        else:
            print(f"[warn] HDF5 里没有 {name}，跳过")
    return picked


def main() -> int:
    ap = argparse.ArgumentParser(description="HDF5 → LeRobot v3（纯 h5py + lerobot）")
    ap.add_argument("--hdf5", type=str, required=True, help="输入 HDF5（IsaacLab/LeIsaac 录制）")
    ap.add_argument("--repo_id", type=str, required=True, help="输出数据集名（目录名）")
    ap.add_argument("--root", type=str, default=None, help="输出根目录；数据集落在 <root>/<repo_id>（默认 lerobot 的 HF 缓存）")
    ap.add_argument("--task", type=str, default="Pick oranges with the leRobot bi-arm manipulator.", help="任务描述字符串（写进每条 frame 的 task）")
    ap.add_argument("--robot_type", type=str, default="bi_so101_follower")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--episodes", type=str, default="all", help="要转哪些 episode：all / '0,1' / '0-3'")
    ap.add_argument("--max_episodes", type=int, default=None, help="最多转几个（测试用）")
    ap.add_argument(
        "--only_success",
        dest="only_success",
        action="store_true",
        default=True,
        help="只转 `attrs['success'] == True` 的 episode（默认；采集流程会用它滤掉"
        "「被丢弃/中断」的残缺集）。要连失败的也转就加 --all_episodes",
    )
    ap.add_argument("--all_episodes", dest="only_success", action="store_false",
                    help="失败的 episode 也转（默认不转）")
    ap.add_argument("--skip_first_frames", type=int, default=5, help="每集跳过前 N 帧（与原脚本一致，默认 5）")
    ap.add_argument("--no_videos", action="store_true", help="用 PNG(image) 存图而不是视频(video)")
    ap.add_argument("--action_align", dest="action_align", action="store_true", default=True, help="当动作维度 == 12 时做 弧度→电机量值 转换（默认）；维度 != 12 时自动关闭")
    ap.add_argument("--no_action_align", dest="action_align", action="store_false", help="动作保持弧度不做转换（只对 12 维动作有效）")
    ap.add_argument("--verify", action="store_true", help="转完重新加载数据集自检（帧数/形状/数值抽样）")
    ap.add_argument(
        "--video_backend",
        type=str,
        default="pyav",
        choices=["pyav", "torchcodec"],
        help="加载视频的解码后端。默认 pyav：本环境的 torchcodec 缺 FFmpeg 共享库（libavutil.so.59）会加载失败，"
        "而 PyAV 自带静态 FFmpeg（pyav 也是编码时用的那个）。训练时对应 --dataset.video_backend=pyav",
    )
    args = ap.parse_args()

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:  # pragma: no cover
        print(f"[convert] ❌ 需要 lerobot：{exc}\n  请用 leisaac-lerobot-sim 环境运行：\n"
              f"  <装了 lerobot 的 python> {sys.argv[0]} ...（见操作指南 §7）")
        return 2

    if not os.path.isfile(args.hdf5):
        print(f"[convert] ❌ 找不到 {args.hdf5}")
        return 2

    root = None
    if args.root:
        root = os.path.abspath(os.path.join(args.root, args.repo_id))
        os.makedirs(os.path.dirname(root), exist_ok=True)

    only_success = bool(args.only_success)

    with h5py.File(args.hdf5, "r") as f:
        episodes = list_episodes(f, args.episodes)
        if args.max_episodes is not None:
            episodes = episodes[: args.max_episodes]
        if not episodes:
            print("[convert] ❌ 没有可转换的 episode")
            return 2
        first = f["data"][episodes[0]]
        action_dim = int(first["actions"].shape[1])
        img = first["obs"]["left_wrist"][0]
        image_shape = tuple(int(v) for v in img.shape)

        # ★ 与上游 `build_feature_from_env()` 同款判据：
        #   动作维度 == 关节数（12）→ 关节空间，用真关节名 + 弧度→电机量值转换；
        #   否则（如双臂键盘 16 维）→ `dim_i` 命名 + **原样存**，因为那些维是笛卡尔位姿增量，
        #   不是关节角，做"弧度→电机量值"映射没有物理意义。
        if action_dim == STATE_DIM:
            action_names = DEFAULT_FEATURE_JOINT_NAMES
            action_align = bool(args.action_align)
            semantics = "关节空间（与 observation.state 同构）"
        else:
            action_names = [f"dim_{i}" for i in range(action_dim)]
            action_align = False
            semantics = f"非关节空间（IK 位姿增量等）→ 原样存，不做数值转换"
            if args.action_align:
                print("[convert] 注意：动作维度 != 12，已自动关闭『弧度→电机量值』转换"
                      "（与上游 build_feature_from_env 的行为一致）")

        print(f"[convert] {args.hdf5}")
        print(f"[convert] episodes={len(episodes)}（{episodes[0]}..{episodes[-1]}） action_dim={action_dim} "
              f"state_dim={STATE_DIM} image_shape={image_shape} fps={args.fps} "
              f"videos={not args.no_videos} align={action_align}")
        print(f"[convert] 动作语义：{semantics}")

        features = build_features(
            image_shape, action_dim, action_names, use_videos=not args.no_videos, fps=args.fps
        )
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            features=features,
            root=root,
            robot_type=args.robot_type,
            use_videos=not args.no_videos,
        )
        print(f"[convert] 数据集根目录：{dataset.root}")

        total_frames = 0
        skipped_failed = 0
        for ep_name in episodes:
            ep = f["data"][ep_name]
            if "actions" not in ep:
                print(f"[convert] {ep_name}: 没有 actions 字段（录制中断的残缺集），跳过")
                continue
            if only_success and not bool(dict(ep.attrs).get("success", False)):
                skipped_failed += 1
                print(f"[convert] {ep_name}: success=False（被丢弃/中断的那条），跳过"
                      f"（要转就加 --all_episodes）")
                continue
            n = int(ep["actions"].shape[0])
            if n < 10:
                print(f"[convert] {ep_name}: 只有 {n} 帧（<10），跳过")
                continue
            actions = ep["actions"][:]
            left_pos = ep["obs"]["left_joint_pos"][:]
            right_pos = ep["obs"]["right_joint_pos"][:]
            cams = {c: ep["obs"][c] for c in CAMERAS if c in ep["obs"]}
            added = 0
            for t in range(args.skip_first_frames, n):
                act = actions[t]
                if action_align:
                    act = convert_leisaac_action_to_lerobot(act[None, :])[0]
                # state 永远是 12 维关节角→电机量值（与录制设备无关）
                state = np.concatenate(
                    [
                        convert_leisaac_action_to_lerobot(left_pos[t][None, :])[0],
                        convert_leisaac_action_to_lerobot(right_pos[t][None, :])[0],
                    ],
                    axis=0,
                )
                frame = {
                    "action": act.astype(np.float32),
                    "observation.state": state.astype(np.float32),
                    "task": args.task,
                }
                for cam, ds in cams.items():
                    frame[f"observation.images.{cam}"] = np.asarray(ds[t], dtype=np.uint8)
                dataset.add_frame(frame)
                added += 1
            dataset.save_episode()
            total_frames += added
            print(f"[convert] {ep_name}: {added} 帧 已保存（累计 {total_frames}）", flush=True)

        # ★ 必须收尾：v3.0 的每集元数据（meta/episodes/**）与 parquet 合并都在这一步写盘。
        #   不调用的话 meta/ 里只有 info.json/stats.json/tasks.parquet，
        #   之后 LeRobotDataset(root=...) 会报 FileNotFoundError 然后**去连 huggingface.co**（本机连不上）。
        dataset.finalize()
        msg = f"[convert] ✅ 完成：{total_frames} 帧 → {dataset.root}"
        if skipped_failed:
            msg += f"（另有 {skipped_failed} 条 success=False 已跳过）"
        print(msg)

    if args.verify:
        print("\n[verify] 重新加载数据集自检 ...")
        ds = LeRobotDataset(repo_id=args.repo_id, root=root, video_backend=args.video_backend)
        print(f"[verify] num_episodes={ds.num_episodes} num_frames={ds.num_frames} fps={ds.fps}")
        print(f"[verify] features={list(ds.features.keys())}")
        for key, feat in ds.features.items():
            print(f"[verify]   {key}: dtype={feat.get('dtype')} shape={tuple(feat.get('shape'))}")
        try:
            from lerobot.datasets.utils import dataset_to_policy_features

            pf = dataset_to_policy_features(ds.features)
            print("[verify] 转成 policy 特征（应与既有 checkpoint 的输入形状一致）：")
            for k, v in pf.items():
                print(f"[verify]   {k}: {v.type} {tuple(v.shape)}")
        except Exception as exc:  # noqa: BLE001
            print(f"[verify] （dataset_to_policy_features 不可用：{exc}）")
        sample = ds[0]
        print(f"[verify] 第 0 帧 keys={list(sample.keys())}")
        for k, v in sample.items():
            if hasattr(v, "shape"):
                print(f"[verify]   {k}: shape={tuple(v.shape)} dtype={getattr(v, 'dtype', None)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
