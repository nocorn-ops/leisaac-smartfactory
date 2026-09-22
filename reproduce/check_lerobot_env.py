#!/usr/bin/env python
"""验证某个 lerobot 环境：① 依赖是否装齐；② 能否加载 ACT checkpoint 并跑出动作块。

用法（在目标 conda 环境里跑）:
    <env>/bin/python reproduce/check_lerobot_env.py --deps-only
        # 只查依赖 —— **装完环境先跑这个**（新机器上可能还没有 checkpoint）
    <env>/bin/python reproduce/check_lerobot_env.py [--checkpoint PATH] [--device cpu|cuda]
        # 再验一遍"能加载模型并推理"
"""
import argparse
import importlib
import importlib.util as _u
import sys

#: 本项目跑「HDF5→LeRobot 转换 / 训练 / 策略服务端」需要的包：(import 名, 安装名)
REQUIRED = [
    ("lerobot", "lerobot[async]==0.4.2"),
    ("torch", "torch==2.7.1+cu128"),
    ("torchvision", "torchvision==0.22.1+cu128"),
    ("h5py", "h5py"),
    ("av", "av"),
    ("numpy", "numpy"),
    ("PIL", "pillow"),
    ("datasets", "datasets"),
    ("grpc", "grpcio"),
]


def check_deps() -> int:
    """逐项 import 检查；返回 0=齐全，1=有缺失。"""
    print("=" * 70)
    print("Python:", sys.version.split()[0], "| prefix:", sys.prefix)
    bad = []
    for mod, pkg in REQUIRED:
        try:
            m = importlib.import_module(mod)
            print(f"  ✅ {mod:<14} {getattr(m, '__version__', ''):<18} ({pkg})")
        except Exception as exc:  # noqa: BLE001
            bad.append(mod)
            print(f"  ❌ {mod:<14} {'':<18} ({pkg})  -> {type(exc).__name__}: {exc}")
    try:
        import torch

        if torch.cuda.is_available():
            print(f"  ℹ️  torch CUDA 可用：{torch.cuda.get_device_name(0)}")
        else:
            print("  ℹ️  torch CUDA 不可用（仿真侧仍能跑，策略服务端可用 --device cpu）")
    except Exception:  # noqa: BLE001
        pass
    for mod in ("lerobot.policies.act", "lerobot.async_inference", "lerobot.datasets.lerobot_dataset"):
        try:
            ok = _u.find_spec(mod) is not None
        except Exception:  # noqa: BLE001
            ok = False
        print(f"  {'✅' if ok else '❌'} {mod}")
        if not ok:
            bad.append(mod)
    print("-" * 70)
    if bad:
        print(f"❌ 缺 {len(bad)} 项：{', '.join(bad)}")
        print("   重装: bash reproduce/install_all.sh --skip-isaacsim --skip-isaaclab")
        return 1
    print("✅ 依赖齐全")
    return 0


parser = argparse.ArgumentParser()
parser.add_argument(
    "--checkpoint",
    type=str,
    default="outputs/train/kitchen_biarm_act_300k/checkpoints/300000/pretrained_model",
)
parser.add_argument("--device", type=str, default="cpu")
parser.add_argument("--deps-only", action="store_true", help="只检查依赖，不加载 checkpoint")
args = parser.parse_args()

_deps_rc = check_deps()
if args.deps_only:
    raise SystemExit(_deps_rc)
if _deps_rc != 0:
    print("❌ 依赖不全，跳过后面的 checkpoint 验证。")
    raise SystemExit(_deps_rc)

print("=" * 70)
print("下一步：验证能否加载 checkpoint ——", args.checkpoint)

import lerobot  # noqa: E402

print("lerobot:", lerobot.__version__)
try:
    from lerobot.policies.act.modeling_act import ACTPolicy

    policy = ACTPolicy.from_pretrained(args.checkpoint)
    policy.to(args.device)
    policy.eval()
    print("✅ ACTPolicy.from_pretrained 成功")
    cfg = policy.config
    for k in ("n_obs_steps", "chunk_size", "n_action_steps", "vision_backbone", "normalization_mapping"):
        print(f"   config.{k} = {getattr(cfg, k, None)}")
except Exception as exc:  # noqa: BLE001
    import traceback

    print("❌ ACTPolicy 加载失败:", type(exc).__name__, exc)
    traceback.print_exc()
    raise SystemExit(1)

print("-" * 70)
pre = post = None
try:
    from lerobot.policies.factory import make_pre_post_processors

    pre, post = make_pre_post_processors(policy.config, pretrained_path=args.checkpoint)
    print("✅ make_pre_post_processors 成功（0.5.x 风格 API）")
except Exception as exc:  # noqa: BLE001
    print("⚠️ make_pre_post_processors 不可用:", type(exc).__name__, exc)

print("-" * 70)
try:
    import numpy as np
    import torch

    rng = np.random.default_rng(0)
    obs = {"observation.state": torch.zeros(1, 12)}
    for cam in ("left_wrist", "right_wrist", "front"):
        obs[f"observation.images.{cam}"] = torch.from_numpy(
            rng.integers(0, 255, (1, 3, 240, 320), dtype=np.uint8)
        ).float() / 255.0
    if pre is not None:
        obs = pre(obs)
        print("   已过 preprocessor")
    policy.reset()
    with torch.inference_mode():
        actions = policy.select_action(obs)
    print(f"✅ select_action 成功：shape={tuple(actions.shape)} range=[{actions.min():.2f}, {actions.max():.2f}]")
except Exception as exc:  # noqa: BLE001
    import traceback

    print("❌ 推理失败:", type(exc).__name__, exc)
    traceback.print_exc()
    raise SystemExit(1)

print("=" * 70)
print("结论：这个环境可以用于本项目的 ACT 推理。")
