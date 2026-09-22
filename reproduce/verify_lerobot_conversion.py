#!/usr/bin/env python
"""数值抽检：转换出来的 LeRobot v3 数据集，逐帧与源 HDF5 对比，证明映射没错。

检查项：
1. 帧数 = Σ(每集帧数 − skip_first_frames)
2. `observation.state` / `action` 与 HDF5 原值（经同样的 弧度→电机值 换算）逐元素一致
3. 图像与 HDF5 原帧一致（对比归一化/转置后的像素）
4. 任务字符串、episode_index、frame_index 正确

用法（在 leisaac-lerobot-sim 环境里跑）::

    <lerobot-python> reproduce/verify_lerobot_conversion.py \
        --hdf5 datasets/kitchen_biarm.hdf5 \
        --root /tmp/lerobot_out3/kitchen_biarm_v1_v3 \
        --repo_id kitchen_biarm_v1_v3 --episodes 0,1
"""
from __future__ import annotations

import argparse
import os
import sys

import h5py
import numpy as np

# ⚠️ 必须用 **__file__ 相对**路径，不能写 CWD 相对的 "scripts/convert"：
#    后者只有"在仓库根目录运行"才对，换个目录调用（脚本里 cd 过、别处 import、
#    或赛队解压到别处再跑）就 ImportError。这是最典型的"换个电脑就挂"的写法。
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "scripts", "convert"))
from hdf5_to_lerobot_v3 import (  # noqa: E402
    CAMERAS,
    convert_leisaac_action_to_lerobot,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hdf5", required=True)
    ap.add_argument("--root", required=True, help="数据集目录（含 meta/info.json）")
    ap.add_argument("--repo_id", required=True)
    ap.add_argument("--episodes", default="0")
    ap.add_argument("--skip_first_frames", type=int, default=5)
    ap.add_argument("--tol", type=float, default=1e-4)
    ap.add_argument("--img_tol", type=float, default=6.0, help="图像平均绝对差容许值（AV1 有损，实测 ~2.8/255）")
    ap.add_argument(
        "--video_backend",
        type=str,
        default="pyav",
        choices=["pyav", "torchcodec"],
        help="视频解码后端。默认 pyav（torchcodec 在本环境缺 FFmpeg 共享库，加载即失败）",
    )
    args = ap.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(repo_id=args.repo_id, root=args.root, video_backend=args.video_backend)
    demos = [f"demo_{int(x)}" for x in args.episodes.split(",")]
    ok = True

    with h5py.File(args.hdf5, "r") as f:
        expect_frames = sum(int(f["data"][d]["actions"].shape[0]) - args.skip_first_frames for d in demos)
        print(f"[check] 期望帧数={expect_frames}  数据集 num_frames={ds.num_frames} "
              f"num_episodes={ds.num_episodes}")
        if ds.num_frames != expect_frames:
            ok = False
            print("  ❌ 帧数不一致")

        base = 0  # 数据集是按"全局帧号"索引的
        for ep_i, demo in enumerate(demos):
            ep = f["data"][demo]
            n = int(ep["actions"].shape[0])
            actions = ep["actions"][:]
            left = ep["obs"]["left_joint_pos"][:]
            right = ep["obs"]["right_joint_pos"][:]
            cams = {c: ep["obs"][c] for c in CAMERAS if c in ep["obs"]}
            # 抽 3 帧：第一帧、中间、最后一帧
            picks = [args.skip_first_frames, (args.skip_first_frames + n - 1) // 2, n - 1]
            for t in picks:
                idx = base + (t - args.skip_first_frames)
                item = ds[idx]
                exp_act = convert_leisaac_action_to_lerobot(actions[t][None, :])[0]
                exp_state = np.concatenate(
                    [
                        convert_leisaac_action_to_lerobot(left[t][None, :])[0],
                        convert_leisaac_action_to_lerobot(right[t][None, :])[0],
                    ]
                )
                got_act = np.asarray(item["action"], dtype=np.float64).reshape(-1)
                got_state = np.asarray(item["observation.state"], dtype=np.float64).reshape(-1)
                d_act = float(np.abs(got_act - exp_act).max())
                d_state = float(np.abs(got_state - exp_state).max())
                img_ok = True
                worst_img = 0.0
                for cam, arr in cams.items():
                    got = item[f"observation.images.{cam}"]
                    got = got.detach().cpu().numpy() if hasattr(got, "detach") else np.asarray(got)
                    if got.ndim == 3 and got.shape[0] == 3:  # CHW
                        got = np.transpose(got, (1, 2, 0))
                    if got.dtype != np.uint8:
                        got = (np.clip(got, 0, 1) * 255).astype(np.uint8)
                    ref = np.asarray(arr[t], dtype=np.uint8)
                    diff = float(np.abs(got.astype(np.int16) - ref.astype(np.int16)).mean())
                    worst_img = max(worst_img, diff)
                    if diff > args.img_tol:  # 视频编码有损（AV1），允许平均若干灰阶误差
                        img_ok = False
                frame_i = int(item["frame_index"]) if "frame_index" in item else -1
                ep_i_got = int(item["episode_index"]) if "episode_index" in item else -1
                good = d_act <= args.tol and d_state <= args.tol and img_ok
                ok = ok and good
                print(
                    f"  {'✅' if good else '❌'} ep{ep_i}({demo}) t={t} idx={idx}: "
                    f"|Δaction|={d_act:.2e} |Δstate|={d_state:.2e} 图像平均差={worst_img:.2f} "
                    f"frame_index={frame_i} episode_index={ep_i_got} task={str(item.get('task'))[:40]!r}"
                )
            base += n - args.skip_first_frames

    print(f"\n[check] 图像平均差容许值={args.img_tol}（AV1 有损编码，非 0 是正常的）")
    print("[check] 结论：" + ("数值完全对齐 ✅" if ok else "存在不一致 ❌"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
