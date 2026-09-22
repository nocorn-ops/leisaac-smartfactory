#!/usr/bin/env python
"""M2 验收：三件货物（香蕉 / 茄子 / 收纳盒）能不能**稳定落在架子上**。

判据（每件货物都要满足）：
  ① 生成后下落并**停住**（末速度 < 0.02 m/s）——没有一直抖/滑动
  ② 停在**正确的层高**上（2026-09-16 起**三件全部在上层台面**，误差 < 2 cm）
     —— 层高从 `smart_factory_layout.py` 的 `TIER_HIGH_Z` 取，不写死
  ③ **没有掉下去**（最终 z 高于台面 - 3 cm）
  ④ 没有穿进台面里（最终 z 高于台面 - 1 cm）
另外报告水平位移（摆放点 → 落点），用来发现"放上去就滑走"。

用法:
    reproduce/verify_smart_factory_props.py                 # 默认静置 300 步（5 秒仿真）
    reproduce/verify_smart_factory_props.py --steps 600
    reproduce/verify_smart_factory_props.py --render /tmp/m2  # 顺带出图（会慢一些）
"""

import argparse
import dataclasses

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--steps", type=int, default=300, help="静置步数（60 步 ≈ 1 秒仿真）")
parser.add_argument("--render", type=str, default=None, help="出图目录（给了就渲染几张）")
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=True, enable_cameras=args_cli.render is not None)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.assets.scenes.smart_factory_layout import (  # noqa: E402
    BANANA_XY,
    BANANA_Z,
    CRATE_XY,
    CRATE_SIZE,
    CRATE_Z,
    EGGPLANT_XY,
    EGGPLANT_Z,
    TIER_HIGH_Z,
)

env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()


def _t(x):
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().cpu().numpy()


def pose(name):
    obj = env.scene[name]
    return _t(obj.data.root_pos_w)[0], _t(obj.data.root_lin_vel_w)[0]


# 期望：落位后的**中心高度**（物体半高 + 台面高度）
EXPECT = {
    # ★ 2026-09-16：三件**全部在上层台面**，沿 Y 从 −Y 到 +Y 依次 茄子 → 收纳盒 → 香蕉
    "banana": dict(spawn=(*BANANA_XY, BANANA_Z), rest=TIER_HIGH_Z + 0.020, tier=TIER_HIGH_Z,
                   what="上层台面 +Y 端"),
    "eggplant": dict(spawn=(*EGGPLANT_XY, EGGPLANT_Z), rest=TIER_HIGH_Z + 0.035, tier=TIER_HIGH_Z,
                     what="上层台面 −Y 端"),
    "crate": dict(spawn=(*CRATE_XY, CRATE_Z), rest=TIER_HIGH_Z + CRATE_SIZE[2] / 2, tier=TIER_HIGH_Z,
                  what="上层台面中间"),
}

# 生成瞬间的位姿
start = {name: pose(name)[0].copy() for name in EXPECT}
for _ in range(args_cli.steps):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

print("=" * 78)
print(f"M2 静置测试：{args_cli.steps} 步（约 {args_cli.steps * env.physics_dt:.1f} s 仿真）")
print("-" * 78)
ok_all = True
for name, exp in EXPECT.items():
    pos, vel = pose(name)
    sp = exp["spawn"]
    dz = float(pos[2] - sp[2])
    dxy = float(((pos[0] - sp[0]) ** 2 + (pos[1] - sp[1]) ** 2) ** 0.5)
    speed = float((vel**2).sum() ** 0.5)
    checks = {
        "停住": speed < 0.02,
        "层高对": abs(pos[2] - exp["rest"]) < 0.02,
        "没掉下去": pos[2] > exp["tier"] - 0.03,
        "没穿模": pos[2] > exp["tier"] - 0.01,
    }
    ok = all(checks.values())
    ok_all = ok_all and ok
    print(f"{name:9s} 落点=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})  "
          f"Δz={dz:+.3f} Δxy={dxy:.3f}  末速={speed:.4f} m/s")
    print(f"          期望中心 z≈{exp['rest']:.3f}（{exp['what']} z={exp['tier']}）  "
          + "  ".join(f"{'✅' if v else '❌'}{k}" for k, v in checks.items()))
print("-" * 78)
print("结论: " + ("✅ 三件货物都稳定落位" if ok_all else "❌ 有不稳定项，见上"))
print("=" * 78)

if args_cli.render:
    import os

    from isaaclab.sensors import TiledCamera  # noqa: F401  (传感器已在场景里)
    import numpy as np
    from PIL import Image

    cam = env.scene.sensors.get("preview")
    views = {
        "props_pickup": ((3.45, 0.85, 0.95), (2.28, 0.85, 0.52)),   # 从东侧（作业方向）看
        "props_side": ((3.35, 2.05, 1.10), (2.28, 0.85, 0.52)),     # 东北 3/4 视角（能看到三件货的样子）
        "props_top": ((2.30, 1.05, 2.20), (2.28, 0.85, 0.50)),      # 俯视货架
        "crate_handle": ((2.62, 0.52, 0.86), (2.30, 1.13, 0.53)),   # 收纳盒特写（提手现在朝 ±Y，从南侧看）
        # 正上方俯视看不到下层台面（会被上层台面挡住），所以用两个低角度机位看长边朝向：
        "crate_east": ((3.05, 1.13, 0.70), (2.29, 1.13, 0.52)),     # 从东侧（机器人作业方向）看
        "crate_south": ((2.29, 0.32, 0.70), (2.29, 1.13, 0.52)),    # 从南侧看
    }
    os.makedirs(args_cli.render, exist_ok=True)
    for vname, (eye, target) in views.items():
        cam.set_world_poses_from_view(
            torch.tensor([eye], device=env.device, dtype=torch.float32),
            torch.tensor([target], device=env.device, dtype=torch.float32),
        )
        for _ in range(8):
            env.sim.render()
            env.scene.update(env.physics_dt)
        arr = cam.data.output.get("rgb")[0].detach().cpu().numpy()
        if arr.dtype != np.uint8:
            arr = (np.nan_to_num(arr[..., :3]).clip(0, 1) * 255).astype(np.uint8)
        path = os.path.join(args_cli.render, f"{vname}.png")
        Image.fromarray(arr[..., :3]).save(path)
        print(f"[shot] 已保存 {path}")

env.close()
simulation_app.close()
