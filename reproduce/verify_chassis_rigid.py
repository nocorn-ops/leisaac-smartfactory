#!/usr/bin/env python
"""验证"底盘转动时机械臂保持与机器人相对静止"（刚体跟随）。

判据：
1. 机械臂相对底盘的**本体系位置偏移** `Rz(-body_yaw)·(arm_pos - body_pos)` 在运动前后**不变**；
2. 机械臂与底盘的**相对朝向** `arm_yaw - body_yaw` 在运动前后**不变**；
3. 底盘 yaw 的实际变化量 == 指令积分值（`wz * dt * N`）。

用法: reproduce/verify_chassis_rigid.py
"""
import dataclasses
import math

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True, enable_cameras=False)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
import omni.usd  # noqa: E402
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.chassis import ChassisController, yaw_of_xyzw  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402
from pxr import UsdGeom  # noqa: E402


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


env_cfg = parse_env_cfg("LeIsaac-LeRobot-Kitchen-v0", device="cuda:0", num_envs=1)
for attr in ("observations", "events"):
    if hasattr(env_cfg, attr):
        _strip(getattr(env_cfg, attr))
for name in list(vars(env_cfg.scene).keys()):
    if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
        setattr(env_cfg.scene, name, None)
env_cfg.use_teleop_device(get_task_type("LeIsaac-LeRobot-Kitchen-v0"))
env_cfg.recorders = None
for _name in ("time_out", "success"):
    if hasattr(env_cfg.terminations, _name):
        setattr(env_cfg.terminations, _name, None)

env = gym.make("LeIsaac-LeRobot-Kitchen-v0", cfg=env_cfg).unwrapped
env.reset()
for _ in range(60):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

chassis = ChassisController(env)
stage = omni.usd.get_context().get_stage()
body_prim = stage.GetPrimAtPath("/World/envs/env_0/Body")
hold = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)

ok = True


def check(name, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  {detail}" if detail else ""))


def body_pose():
    # ⚠️ 必须每次新建 XformCache：ClearXformOpOrder + Add*Op 重建 op 之后，
    #    复用同一个 cache 会读到**旧值**（踩过，导致误判"底盘没动"）。
    world = UsdGeom.XformCache().GetLocalToWorldTransform(body_prim)
    t = world.ExtractTranslation()
    q = world.ExtractRotationQuat()
    quat = np.array([q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()])
    return np.array([t[0], t[1], t[2]]), yaw_of_xyzw(quat)


def arm_pose(name):
    arm = env.scene[name]
    p = arm.data.root_pos_w
    p = p.torch if hasattr(p, "torch") else p
    q = arm.data.root_quat_w
    q = q.torch if hasattr(q, "torch") else q
    return p[0].detach().cpu().numpy().astype(np.float64), yaw_of_xyzw(q[0].detach().cpu().numpy())


def snapshot():
    bpos, byaw = body_pose()
    out = {}
    for name in ("left_arm", "right_arm"):
        apos, ayaw = arm_pose(name)
        d = apos - bpos
        c, s = math.cos(-byaw), math.sin(-byaw)
        out[name] = {
            "offset_body": np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]]),
            "rel_yaw": (ayaw - byaw + math.pi) % (2 * math.pi) - math.pi,
            "dist": float(np.linalg.norm(d)),
        }
    return out, bpos, byaw


def spin(wz, seconds, vx=0.0, vy=0.0, dt=1.0 / 30.0):
    steps = int(round(seconds / dt))
    for _ in range(steps):
        chassis.step(vx, vy, wz, dt)
        env.step(hold)
        env.scene.update(env.physics_dt)
    return steps


print("\n===== 底盘转动时机械臂是否保持相对静止")
s0, b0, y0 = snapshot()
print(f"  初始：body=({b0[0]:.3f},{b0[1]:.3f}) yaw={math.degrees(y0):.1f}°")
for n, v in s0.items():
    print(f"    {n}: 本体系偏移=({v['offset_body'][0]:+.3f},{v['offset_body'][1]:+.3f},{v['offset_body'][2]:+.3f}) "
          f"相对朝向={math.degrees(v['rel_yaw']):+.1f}° 距底盘={v['dist']:.3f}m")

# ── 1) 原地转 +90° ──
steps = spin(wz=1.0, seconds=math.pi / 2)
s1, b1, y1 = snapshot()
dyaw = math.degrees((y1 - y0 + math.pi) % (2 * math.pi) - math.pi)
print(f"\n  原地转 {steps} 步后：yaw={math.degrees(y1):.1f}°（Δ={dyaw:+.1f}°）")
check("底盘转了约 +90°", abs(dyaw - 90.0) < 3.0, f"Δ={dyaw:+.1f}°")
for n in s0:
    d_off = float(np.abs(s1[n]["offset_body"] - s0[n]["offset_body"]).max())
    d_rel = abs(math.degrees((s1[n]["rel_yaw"] - s0[n]["rel_yaw"] + math.pi) % (2 * math.pi) - math.pi))
    check(
        f"{n} 转动后本体系偏移不变",
        d_off < 1e-3,
        f"最大变化 {d_off * 1000:.2f} mm",
    )
    check(f"{n} 转动后相对朝向不变", d_rel < 0.5, f"变化 {d_rel:.2f}°")

# ── 2) 边走边转（平移 + 旋转）──
s2, b2, y2 = snapshot()
steps = spin(vx=0.25, wz=-0.8, seconds=2.0)
s3, b3, y3 = snapshot()
moved = float(np.linalg.norm(b3 - b2))
dyaw2 = math.degrees((y3 - y2 + math.pi) % (2 * math.pi) - math.pi)
print(f"\n  平移+旋转 {steps} 步：位移 {moved:.3f} m，yaw Δ={dyaw2:+.1f}°")
check("边走边转时底盘确实移动了", moved > 0.2, f"{moved:.3f} m")
for n in s0:
    d_off = float(np.abs(s3[n]["offset_body"] - s2[n]["offset_body"]).max())
    d_rel = abs(math.degrees((s3[n]["rel_yaw"] - s2[n]["rel_yaw"] + math.pi) % (2 * math.pi) - math.pi))
    check(f"{n} 复合运动后本体系偏移不变", d_off < 1e-3, f"最大变化 {d_off * 1000:.2f} mm")
    check(f"{n} 复合运动后相对朝向不变", d_rel < 0.5, f"变化 {d_rel:.2f}°")

print("\n===== 结论：" + ("机械臂与底盘保持刚体跟随 ✅" if ok else "存在偏移 ❌"))
env.close()
simulation_app.close()
raise SystemExit(0 if ok else 1)
