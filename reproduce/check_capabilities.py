#!/usr/bin/env python
"""能力确认：LiDAR(RayCaster) 是否真出数据 + 底盘能否被移动 + 双臂关节是否可读。

这三个都是"导航/操作能力"的底层前提，之前从未单独验证过。
用法: reproduce/check_capabilities.py [--task ...]
"""
import argparse
import dataclasses

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = False
args_cli.headless = True
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
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


env_cfg = parse_env_cfg(args_cli.task, device="cuda:0", num_envs=1)
for attr in ("observations", "events"):
    if hasattr(env_cfg, attr):
        _strip(getattr(env_cfg, attr))
for name in list(vars(env_cfg.scene).keys()):
    if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
        setattr(env_cfg.scene, name, None)
env_cfg.use_teleop_device(get_task_type(args_cli.task))
env_cfg.recorders = None
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()
for _ in range(30):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

print("\n===== 1) LiDAR（RayCaster）")
lidar = env.scene["lidar"]
print(f"  prim={lidar.cfg.prim_path}  rays={lidar.cfg.pattern_cfg.horizontal_res}/° "
      f"fov={lidar.cfg.pattern_cfg.horizontal_fov_range} max_distance={lidar.cfg.max_distance}")
pos = lidar.data.pos_w
hits = lidar.data.ray_hits_w
pos = pos.torch if hasattr(pos, "torch") else pos
hits = hits.torch if hasattr(hits, "torch") else hits
hits_np = hits[0].detach().cpu().numpy()
finite = np.isfinite(hits_np).all(axis=1)
dist = np.linalg.norm(hits_np - pos[0].detach().cpu().numpy(), axis=1)
print(f"  传感器位置={[round(float(v), 3) for v in pos[0]]}")
print(f"  命中点数={len(hits_np)}  有限命中={int(finite.sum())}（inf=没打中）")
if finite.any():
    d = dist[finite]
    print(f"  距离范围=[{d.min():.3f}, {d.max():.3f}] m  中位数={np.median(d):.3f} m  → "
          f"{'✅ LiDAR 有有效数据' if d.min() < lidar.cfg.max_distance else '❌ 数据可疑'}")

print("\n===== 2) 底盘（body）能否被移动 + 双臂是否跟随")
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

body_prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/envs/env_0/Body")
print(f"  body prim={body_prim.GetPath() if body_prim else None} 类型={body_prim.GetTypeName()}")
print(f"  scene 里的 body 类型={type(env.scene['body']).__name__}（静态视觉模型，无 RigidBodyAPI → 运动学传送）")
left, right = env.scene["left_arm"], env.scene["right_arm"]


def _t(x):
    x = x.torch if hasattr(x, "torch") else x
    return x.detach().clone()


before = {}
for n in ("left_arm", "right_arm"):
    before[n] = _t(env.scene[n].data.root_pos_w)[0]
# 用与 scripts/test_chassis_move.py 相同的方式平移底盘（+0.3m 沿 x）
xform = UsdGeom.Xformable(body_prim)
xform.ClearXformOpOrder()
t_op = xform.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat, "xformOp:translate")
t_op.Set(Gf.Vec3d(2.29177 + 0.3, -0.84691, 0.01))
xform.AddRotateXYZOp(UsdGeom.XformOp.PrecisionFloat, "xformOp:rotateXYZ").Set(Gf.Vec3f(0.0, 0.0, 0.0))
for _ in range(5):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
print(f"  平移底盘 +0.3m 后（底盘本身不经过物理，臂按位移跟随）：")
for n in ("left_arm", "right_arm"):
    arm = env.scene[n]
    cur = _t(arm.data.root_pos_w)[0]
    # 臂需要被显式更新（test 脚本里是 write_root_pose_to_sim）
    new_pos = cur.clone()
    new_pos[0] = before[n][0] + 0.3
    quat = _t(arm.data.root_quat_w)[0]
    arm.write_root_pose_to_sim(torch.cat([new_pos, quat]).unsqueeze(0))
for _ in range(5):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
for n in ("left_arm", "right_arm"):
    cur = _t(env.scene[n].data.root_pos_w)[0]
    moved = float((cur - before[n]).norm())
    print(f"    {n}: 位移 {moved:.3f} m → {'✅ 可随底盘移动' if moved > 0.25 else '❌ 没跟上'}")

print("\n===== 3) 双臂关节读数 / 动作维度")
print(f"  action_space={env.action_space}")
print(f"  left 关节数={len(left.cfg.joint_names) if hasattr(left.cfg, 'joint_names') else '?'} "
      f"joint_pos={[round(float(v), 3) for v in _t(left.data.joint_pos)[0]]}")
print(f"  right 关节数={len(right.cfg.joint_names) if hasattr(right.cfg, 'joint_names') else '?'} "
      f"joint_pos={[round(float(v), 3) for v in _t(right.data.joint_pos)[0]]}")

print("\n===== 4) 环境里可用的场景资产")
names = list(vars(env_cfg.scene).keys())
print(f"  scene 属性: {names}")
print(f"  传感器: {list((getattr(env.scene, 'sensors', {}) or {}).keys())}")

env.close()
simulation_app.close()
