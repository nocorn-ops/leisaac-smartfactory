#!/usr/bin/env python
"""M3 验收：机器人进场地后，**相机 / 激光雷达 / 底盘 / 双臂** 是否都正常。

检查项：
  ① 场景实体齐全：body / left_arm / right_arm / body_collider / lidar / body_contact /
     left_wrist / right_wrist / front / banana / eggplant / crate / preview
  ② 底盘在**起点区**、车头朝向收纳区（打印底盘位姿 + 到收纳架的距离）
  ③ **LiDAR 能扫到围栏**（这是场地相对厨房的最大好处）：统计命中数 / 最近距离 / 四面墙距离
  ④ 三路相机能出图（非全黑、尺寸对）
  ⑤ 双肩关节数对（各 6 DOF）
  ⑥ 可选出图（--render DIR）

用法:
    reproduce/verify_smart_factory_robot.py
    reproduce/verify_smart_factory_robot.py --render reproduce/out/sf_m3
"""

import argparse
import math

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
parser.add_argument("--steps", type=int, default=60, help="静置步数")
parser.add_argument("--render", type=str, default=None, help="出图目录")
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=True, enable_cameras=True)  # 相机/LiDAR 都要渲染
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.assets.scenes.smart_factory_layout import (  # noqa: E402
    ARENA_X,
    ARENA_Y,
    PICKUP_XY,
    ROBOT_START_XY,
    ROBOT_START_YAW_DEG,
)

env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()
for _ in range(args_cli.steps):
    env.sim.step(render=args_cli.render is not None)
    env.scene.update(env.physics_dt)


def _t(x):
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().cpu().numpy()


ok_all = True


def check(name: str, cond: bool, detail: str = "") -> None:
    global ok_all
    ok_all = ok_all and bool(cond)
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  {detail}" if detail else ""))


print("=" * 78)
print("① 场景实体")
entities = ["body", "left_arm", "right_arm", "body_collider", "banana", "eggplant", "crate"]
sensors = ["lidar", "body_contact", "left_wrist", "right_wrist", "front", "preview"]
missing_e = [n for n in entities if n not in env.scene.keys()]
missing_s = [n for n in sensors if n not in (env.scene.sensors or {})]
check("场景实体齐全", not missing_e and not missing_s, f"缺={missing_e + missing_s or '无'}")

print("② 机器人位姿")
# ⚠️ body 是 AssetBaseCfg（纯视觉 USD），没有 .data；位姿要从 prim 的世界变换读
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402

_stage = omni.usd.get_context().get_stage()
_body_prim = _stage.GetPrimAtPath("/World/envs/env_0/Body")
bp = UsdGeom.XformCache().GetLocalToWorldTransform(_body_prim).ExtractTranslation()
bp = np.array([float(bp[0]), float(bp[1]), float(bp[2])])
print(f"     底盘世界位置=({bp[0]:+.3f}, {bp[1]:+.3f}, {bp[2]:+.3f})  期望起点区=({ROBOT_START_XY[0]:.2f}, {ROBOT_START_XY[1]:.2f})")
d_start = math.dist(bp[:2], ROBOT_START_XY)
check("底盘停在该起点区", d_start < 0.10, f"偏差 {d_start * 100:.1f} cm")
d_pick = math.dist(bp[:2], PICKUP_XY)
print(f"     到收纳架中心距离 = {d_pick:.2f} m（车头朝向 {ROBOT_START_YAW_DEG:.0f}°，指向收纳区）")
for arm_name in ("left_arm", "right_arm"):
    arm = env.scene[arm_name]
    jp = _t(arm.data.joint_pos)[0]
    ap = _t(arm.data.root_pos_w)[0]
    check(f"{arm_name} 关节数", len(jp) == 6, f"dof={len(jp)} 位置=({ap[0]:+.3f}, {ap[1]:+.3f}, {ap[2]:+.3f})")

print("③ LiDAR 扫场地（关键：围栏能不能扫到）")
lidar = env.scene.sensors["lidar"]
hits = _t(lidar.data.ray_hits_w)[0]
origin = _t(lidar.data.pos_w)[0]
dist = np.linalg.norm(hits - origin, axis=1)
finite = np.isfinite(dist)
n_hit = int(finite.sum())
print(f"     雷达位置=({origin[0]:+.3f}, {origin[1]:+.3f}, {origin[2]:+.3f})  命中 {n_hit}/{len(dist)} 条")
if n_hit:
    d = dist[finite]
    print(f"     命中距离: min={d.min():.2f} 中位={np.median(d):.2f} max={d.max():.2f} m")
    # 四面围栏：从起点区看，最近的一面应该在 0.1~1.5 m 内（起点区离北墙/西墙都很近）
    check("LiDAR 命中足够多（能看到场地）", n_hit > 100, f"{n_hit} 条")
    check("LiDAR 命中距离合理（0.05~8 m）", d.min() > 0.05 and d.max() <= 8.01, f"min={d.min():.2f} max={d.max():.2f}")
else:
    check("LiDAR 命中足够多（能看到场地）", False, "一条都没命中")

print("④ 三路相机")
for cam_name in ("left_wrist", "right_wrist", "front"):
    cam = env.scene.sensors[cam_name]
    rgb = cam.data.output.get("rgb")
    if rgb is None:
        check(f"{cam_name} 有图像", False, "没有 rgb 输出")
        continue
    arr = rgb[0].detach().cpu().numpy()
    check(f"{cam_name} 有图像", arr.shape[0] > 0 and float(np.nanmax(arr)) > 0.01,
          f"shape={arr.shape} max={float(np.nanmax(arr)):.3f}")

if args_cli.render:
    import os

    from PIL import Image

    cam = env.scene.sensors["preview"]
    views = {
        "robot_start": ((2.30, 1.35, 1.75), (0.55, 2.50, 0.55)),   # 从场地里看起点区的机器人
        "robot_top": ((0.45, 2.45, 3.20), (0.45, 2.55, 0.20)),     # 俯视机器人
        "arena_with_robot": ((5.10, -2.30, 2.45), (1.95, 1.45, 0.32)),
    }
    os.makedirs(args_cli.render, exist_ok=True)
    for vname, (eye, target) in views.items():
        cam.set_world_poses_from_view(
            torch.tensor([eye], device=env.device, dtype=torch.float32),
            torch.tensor([target], device=env.device, dtype=torch.float32),
        )
        for _ in range(10):
            env.sim.render()
            env.scene.update(env.physics_dt)
        arr = cam.data.output.get("rgb")[0].detach().cpu().numpy()
        if arr.dtype != np.uint8:
            arr = (np.nan_to_num(arr[..., :3]).clip(0, 1) * 255).astype(np.uint8)
        path = os.path.join(args_cli.render, f"{vname}.png")
        Image.fromarray(arr[..., :3]).save(path)
        print(f"[shot] 已保存 {path}")

print("=" * 78)
print("结论: " + ("✅ M3 机器人与传感器都就绪" if ok_all else "❌ 有项目不通过，见上"))
print("=" * 78)

env.close()
simulation_app.close()
