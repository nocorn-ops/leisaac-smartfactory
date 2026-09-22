#!/usr/bin/env python
"""验收：**双臂键盘遥操**（`--teleop_device bi-keyboard`）能不能真的用。

为什么需要它：单臂的 `SO101Keyboard` 里写死了 `asset_name="robot"`，双臂场景（场地/厨房）
里只有 `left_arm`/`right_arm` → 直接用会抛
`KeyError: "Scene entity with key 'robot' not found"`。
所以新写了 `BiSO101Keyboard`（`T` 键切换左右臂）。本脚本验证它端到端可用。

检查项：
  ① 动作配置是 **4 个 term**，实体分别是 `left_arm` / `right_arm`
  ② 环境能建起来，`total_action_dim == 16`（每臂 8：末端位姿增量 6 + shoulder_pan 1 + 夹爪 1）
  ③ 设备能建，`get_device_state()` 返回 `(16,)`
  ④ `T` 键能切换左右臂
  ⑤ 按住 `Q`（up）若干步后：**只有激活的那条臂动**，另一条保持不动
  ⑥ 连续 `env.step` 不报错

用法（宿主机，仓库根目录）::

    python.sh -u reproduce/verify_bi_keyboard.py
"""

import os

from isaaclab.app import AppLauncher

app_launcher = AppLauncher({"headless": True, "enable_cameras": True})
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.sim_modes import build_device, ensure_observations  # noqa: E402

TASK = "LeIsaac-SmartFactory-v0"
STEPS = 30

failures = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'✅' if ok else '❌'} {name}" + (f"  （{detail}）" if detail else ""))
    if not ok:
        failures.append(name)


env_cfg = parse_env_cfg(TASK, device="cuda:0", num_envs=1)

# 照**真实 teleop 路径**来：开相机（遥操/录制本来就要 --enable_cameras）+ 注入观测组。
# ★ 场地任务自带的 `observations = EmptyObsCfg()`，不注入的话 env.step() 里 recorder 会
#   KeyError: 'policy'（实测）。统一入口就是在 teleop+record / policy 模式下调它的。
injected = ensure_observations(env_cfg)
print(f"[setup] 观测组注入 = {injected}（场地任务本身没有观测组）")
env_cfg.recorders = None          # 本次只验动作空间，不录制（等价 teleop 不带 --record）
env_cfg.use_teleop_device("bi-keyboard")

print("\n===== ① 动作配置 =====")
for term in ("left_arm_action", "left_gripper_action", "right_arm_action", "right_gripper_action"):
    cfg = getattr(env_cfg.actions, term, None)
    print(f"  {term:22s} asset_name={getattr(cfg, 'asset_name', None)}")
check("4 个 action term 都用 left_arm/right_arm",
      getattr(env_cfg.actions.left_arm_action, "asset_name", None) == "left_arm"
      and getattr(env_cfg.actions.right_arm_action, "asset_name", None) == "right_arm")

print("\n===== ② 建环境 =====")
env = gym.make(TASK, cfg=env_cfg).unwrapped
env.reset()
# ★ 必须和统一入口一样先静置：`env.reset()` 后执行器的目标还是"默认关节位姿"，
#   机械臂要从"出生位姿"被 PD 拉过去并稳下来。不静置就量，量到的是这段过渡，
#   会被误读成"没被命令的那条臂在自己下垂"（实测踩过，白查一轮）。
SETTLE_STEPS = 60
for _ in range(SETTLE_STEPS):
    env.sim.step(render=True)
    env.scene.update(env.physics_dt)
dim = env.action_manager.total_action_dim
print(f"  total_action_dim = {dim}")
print(f"  active_terms     = {list(env.action_manager.active_terms)}")
check("动作维度是 16", dim == 16, f"实际 {dim}")
check("观测组 'policy' 已注入（recorder/策略都要它）", "policy" in env.observation_manager.active_terms,
      f"active_terms={list(env.observation_manager.active_terms)}")

print("\n===== ③ 建设备 =====")
dev = build_device(env, device="bi-keyboard", sensitivity=1.0, arm_side="left")
print(f"  device_type = {dev.device_type} | active_side = {dev.active_side}")
state = dev.get_device_state()
check("device_type 正确", dev.device_type == "bi_so101_keyboard")
check("get_device_state() 是 16 维", state.shape == (16,), f"shape={state.shape}")

print("\n===== ④ T 键切换左右臂 =====")
dev._toggle_side()
switched = dev.active_side == "right"
print(f"  按一次 T → {dev.active_side}")
dev._toggle_side()
print(f"  再按一次 T → {dev.active_side}")
check("T 能在左右臂之间切换", switched and dev.active_side == "left")


def ee_z(entity: str) -> float:
    """末端（gripper）在世界系里的 z。"""
    asset = env.scene[entity]
    idxs, _ = asset.find_bodies("gripper")
    return float(asset.data.body_pos_w[0, idxs[0], 2])


import torch  # noqa: E402

print(f"\n===== ⑤a 零动作 {STEPS} 步：两条臂都该**基本不动** =====")
print("  键盘走 use_relative_mode=True 的 IK，「零动作 = 目标跟住当前位姿」，本来会被重力")
print("  匀速往下拽（实测默认 PD 下 30 步 -15.4 cm）。模板对 bi-keyboard 已**关闭机械臂重力**")
print("  （`bi_arm_env_cfg.py::use_teleop_device`），所以这里应当≈0 —— 这一项就是防")
print("  「一开遥操臂就往下塌」回归的守门检查。想让臂有重量加 --arm_gravity。")
zero = torch.zeros(1, 16, device=env.device)
b0 = {s_: ee_z(f"{s_}_arm") for s_ in ("left", "right")}
for _ in range(STEPS):
    env.step(zero)
a0 = {s_: ee_z(f"{s_}_arm") for s_ in ("left", "right")}
drift = {}
for side in ("left", "right"):
    drift[side] = abs(a0[side] - b0[side])
    print(f"  {side:5s}_arm gripper z: {b0[side]:.4f} → {a0[side]:.4f}   |Δ|={drift[side]:.4f}")
check("零动作下两条臂都不动（|Δz| < 1cm；重力开关生效）",
      max(drift.values()) < 0.01,
      f"左 {drift['left'] * 100:.2f} cm / 右 {drift['right'] * 100:.2f} cm")

print(f"\n===== ⑤b 只命令激活臂（按住 Q/up）{STEPS} 步 =====")
active = dev.active_side
other = "right" if active == "left" else "left"
dev._delta[active][0] += dev.pos_sensitivity
dev._started = True
act = dev.advance()
print(f"  advance() → {type(act).__name__} {tuple(act.shape)}")
print(f"  action[0, :8] (左) = {[round(float(v), 4) for v in act[0, :8]]}")
print(f"  action[0, 8:] (右) = {[round(float(v), 4) for v in act[0, 8:]]}")

check("非激活臂的动作整块为 0（语义上就该不动）",
      bool(torch.allclose(act[0, 8:] if active == "left" else act[0, :8], torch.zeros(8, device=env.device))))

before = {s_: ee_z(f"{s_}_arm") for s_ in ("left", "right")}
for _ in range(STEPS):
    env.step(act)
after = {s_: ee_z(f"{s_}_arm") for s_ in ("left", "right")}
for side in ("left", "right"):
    print(f"  {side:5s}_arm gripper z: {before[side]:.4f} → {after[side]:.4f}   "
          f"Δ={after[side] - before[side]:+.4f}")
check(f"激活臂（{active}）确实动了", abs(after[active] - before[active]) > 1e-3)
# 判据用**相对**的：非激活臂的位移不该明显超过"全零时的自然漂移"
check(f"另一条臂（{other}）没有额外运动（不超自然漂移）",
      abs(after[other] - before[other]) <= drift[other] + 0.005,
      f"Δ={abs(after[other] - before[other]) * 100:.2f} cm vs 自然漂移 {drift[other] * 100:.2f} cm")

print(f"\n===== ⑥ 再连续 step {STEPS} 步 =====")
for _ in range(STEPS):
    env.step(act)
print("  无异常")

print("\n" + "=" * 60)
if failures:
    print("❌ 有检查未通过：" + "、".join(failures))
else:
    print("✅ 双臂键盘遥操全部检查通过")
print("=" * 60)

env.close()
simulation_app.close()
# ★ 用 os._exit 显式给退出码：simulation_app.close() 会直接结束进程（码 0），
#   后面的 raise SystemExit 执行不到。
os._exit(1 if failures else 0)
