#!/usr/bin/env python
"""验证底盘碰撞体：
A. 结构：场景里有 `body_collider`（运动学刚体 + 碰撞 API）与 `body_contact` 接触传感器；
B. 功能：把物体放到地面、底盘往前开 → 物体被**推开**、接触力被检测到、保险杠挡住继续顶；
C. LiDAR 前向急停：前方净空小于阈值时不允许继续前进。

用法: reproduce/verify_chassis_collision.py
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
from leisaac.utils.chassis import ChassisController  # noqa: E402
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


env_cfg = parse_env_cfg("LeIsaac-LeRobot-Kitchen-v0", device="cuda:0", num_envs=1)
for attr in ("observations", "events"):
    if hasattr(env_cfg, attr):
        _strip(getattr(env_cfg, attr))
for name in list(vars(env_cfg.scene).keys()):
    if isinstance(getattr(env_cfg.scene, name), TiledCameraCfg):
        setattr(env_cfg.scene, name, None)
env_cfg.use_teleop_device(get_task_type("LeIsaac-LeRobot-Kitchen-v0"))
env_cfg.recorders = None
for _n in ("time_out", "success"):
    if hasattr(env_cfg.terminations, _n):
        setattr(env_cfg.terminations, _n, None)

env = gym.make("LeIsaac-LeRobot-Kitchen-v0", cfg=env_cfg).unwrapped
env.reset()
for _ in range(60):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

sim = env.sim
dt = env.physics_dt
hold = torch.zeros(1, env.action_manager.total_action_dim, device=env.device)
ok = True


def check(name, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  {detail}" if detail else ""))


def step_chassis(chassis, vx, vy, wz, seconds):
    n = int(round(seconds / dt))
    peak_force = 0.0
    blocked_any = False
    for _ in range(n):
        chassis.step(vx, vy, wz, dt)
        env.step(hold)
        env.scene.update(dt)
        peak_force = max(peak_force, chassis.last_contact_force)
        blocked_any = blocked_any or chassis.blocked
    return peak_force, blocked_any


print("\n===== A. 碰撞体结构")
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/envs/env_0/BodyCollider")
check("场景里有 BodyCollider prim", bool(prim and prim.IsValid()))
from pxr import Usd  # noqa: E402

has_col = False
if prim and prim.IsValid():
    # 碰撞 API 通常在子 prim（形状）上，不一定是根 prim
    for p_ in Usd.PrimRange(prim):
        if p_.HasAPI("PhysicsCollisionAPI") or p_.HasAPI("PhysicsMeshCollisionAPI"):
            has_col = True
            break
has_rb = bool(prim) and prim.HasAPI("PhysicsRigidBodyAPI")
kin = False
if prim and prim.IsValid():
    attr = prim.GetAttribute("physics:kinematicEnabled")
    kin = bool(attr.Get()) if attr and attr.IsValid() else False
check("BodyCollider 有碰撞 API", has_col)
check("BodyCollider 是刚体且 kinematic（不会被物理推动）", has_rb and kin, f"rigid={has_rb} kinematic={kin}")
check("场景里有 body_contact 接触传感器", "body_contact" in (env.scene.sensors or {}))

chassis = ChassisController(env, lidar_stop_distance=0.0)
check("ChassisController 识别到碰撞体", chassis.has_collider)
check("默认不做 LiDAR 急停（0=关）", chassis.lidar_stop_distance == 0.0)

print("\n===== B. 碰撞体是否真的挡得住/推得动（底盘在地面高度，台面上的东西够不到）")
x, y, yaw = chassis.pose
fwd = np.array([math.cos(yaw), math.sin(yaw)])
# 放到**机器人身后**的开阔地面（身前是台面/橱柜，会被卡住）
target = np.array([x, y]) - fwd * 0.9
orange = env.scene["Orange002"]
q = orange.data.root_quat_w[0].tolist()
pose = torch.tensor([[target[0], target[1], 0.12, *q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], device=env.device, dtype=torch.float32)
orange.write_root_state_to_sim(pose)
for _ in range(60):
    env.step(hold)
    env.scene.update(dt)
before = orange.data.root_pos_w[0].detach().cpu().numpy().copy()
print(f"  橘子放在机器人身后地面 ({before[0]:.3f},{before[1]:.3f},{before[2]:.3f})")

# B1：先关掉保险杠（阈值设很大）→ 验证"碰撞体确实接触并把物体顶开"
chassis.collision_force_threshold = 1e9
pos_b = chassis.pos.copy()
peak1, _ = step_chassis(chassis, vx=-0.25, vy=0.0, wz=0.0, seconds=3.0)
chassis_moved = float(np.linalg.norm(chassis.pos - pos_b))
after = orange.data.root_pos_w[0].detach().cpu().numpy()
delta = after[:2] - before[:2]
pushed = float(np.linalg.norm(delta))
along = float(np.dot(delta, -fwd))  # 沿"倒车方向"的位移
print(f"  [保险杠关] 倒车 3 秒：底盘位移 {chassis_moved:.3f} m；橘子位移 {pushed:.3f} m"
      f"（沿推挤方向 {along:.3f} m），接触力峰值 {peak1:.1f} N")
check(
    "碰撞体确实接触到了物体（接触力显著 + 物体被顶开）",
    peak1 > 100.0 and pushed > 0.03,
    f"峰值力 {peak1:.0f} N，橘子位移 {pushed:.3f} m",
)
print("     说明：底盘是**运动学传送**，每一步会「顶进」物体再由 PhysX 弹开，"
      "所以表现为「硬顶开」而不是「平顺推动」；要平顺推挤需要把底盘改成动力学刚体。")

# B2：打开保险杠（20N）→ 继续顶住，应该"撞到就停"
chassis.collision_force_threshold = 20.0
pos_a = chassis.pos.copy()
peak2, blocked2 = step_chassis(chassis, vx=-0.25, vy=0.0, wz=0.0, seconds=2.0)
moved2 = float(np.linalg.norm(chassis.pos - pos_a))
print(f"  [保险杠开] 继续倒车 2 秒：底盘位移 {moved2:.3f} m，接触力峰值 {peak2:.1f} N，触发={blocked2}")
check("接触力超阈值时保险杠挡住继续顶", blocked2 and peak2 > 20.0, f"位移 {moved2:.3f} m，峰值 {peak2:.1f} N")

print("\n===== C. LiDAR 前向急停（静态障碍物理上挡不住运动学底盘）")
chassis2 = ChassisController(env, lidar_stop_distance=0.6)
pos_before = chassis2.pos.copy()
blocked_any = False
for _ in range(int(6.0 / dt)):  # 一直往前开 6 秒，应该被"前方净空"拦住
    chassis2.step(0.25, 0.0, 0.0, dt)
    env.step(hold)
    env.scene.update(dt)
    blocked_any = blocked_any or chassis2.blocked
moved = float(np.linalg.norm(chassis2.pos - pos_before))
print(f"  一直往前开 6 秒（会开到台面）：位移 {moved:.3f} m（不拦的话应 ≈1.5 m）；"
      f"最终前方净空 {chassis2.last_forward_clearance:.3f} m；急停触发={blocked_any}")
check(
    "LiDAR 前向急停生效：停在阈值处，没有撞上去",
    blocked_any and moved < 1.4 and chassis2.last_forward_clearance >= chassis2.lidar_stop_distance - 0.05,
    f"位移 {moved:.3f} m（无拦应 1.5 m），净空停在 {chassis2.last_forward_clearance:.3f} m（阈值 {chassis2.lidar_stop_distance}）",
)

print("\n===== 结论：" + ("底盘碰撞体工作正常 ✅" if ok else "有问题 ❌"))
env.close()
simulation_app.close()
raise SystemExit(0 if ok else 1)
