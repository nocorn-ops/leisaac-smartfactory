#!/usr/bin/env python
"""验证动作安全层：位置限幅 / 每步增量限幅 / NaN 守卫 / 形状守卫 / 声明式 clip 兜底。

用法（headless，约 1 分钟）:
    reproduce/verify_action_safety.py
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
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.action_safety import (  # noqa: E402
    ActionSafetyLimiter,
    build_declarative_clip,
    install_declarative_clip,
)
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
for _ in range(20):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

DIM = env.action_manager.total_action_dim
ok = True


def check(name: str, cond: bool, detail: str = "") -> None:
    global ok
    ok = ok and cond
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  {detail}" if detail else ""))


print(f"\n===== 动作安全层验证（dim={DIM}）")
limiter = ActionSafetyLimiter(env, margin_deg=2.0, max_delta=0.1)
limiter.reset()
lo, hi = limiter._lo, limiter._hi
print(f"  限位（内缩后）：lower={[round(float(v), 3) for v in lo[:6]]} …")
print(f"                  upper={[round(float(v), 3) for v in hi[:6]]} …")

# ── 1) 位置限幅：给一个远超行程的动作 ──
out = limiter.filter(torch.full((1, DIM), 10.0, device=env.device))
check("位置限幅：超程动作被裁到上限内", bool((out <= hi + 1e-6).all()), f"max(out)={float(out.max()):.3f}")
out_low = limiter.filter(torch.full((1, DIM), -10.0, device=env.device))
check("位置限幅：负向超程同样被裁", bool((out_low >= lo - 1e-6).all()), f"min(out)={float(out_low.min()):.3f}")

# ── 2) 每步增量限幅 ──
limiter.reset(init_action=torch.zeros(1, DIM, device=env.device))
step_out = limiter.filter(torch.ones(1, DIM, device=env.device) * 5.0)
delta = float((step_out - 0.0).abs().max())
check("增量限幅：一步最多 0.1 rad", delta <= 0.1 + 1e-6, f"实际 Δmax={delta:.4f}")

# ── 3) NaN 守卫 ──
limiter.reset(init_action=torch.full((1, DIM), 0.5, device=env.device))
nan_action = torch.full((1, DIM), 0.5, device=env.device)
nan_action[0, 3] = float("nan")
nan_action[0, 7] = float("inf")
nan_out = limiter.filter(nan_action)
check("NaN/Inf 守卫：非有限维度保持上一步", bool(torch.isfinite(nan_out).all()))
check(
    "NaN/Inf 守卫：有限维度不受影响",
    abs(float(nan_out[0, 0]) - 0.5) < 1e-6 and abs(float(nan_out[0, 3]) - 0.5) < 1e-6,
    f"out[0]={float(nan_out[0, 0]):.3f} out[3]={float(nan_out[0, 3]):.3f}",
)

# ── 4) 形状守卫 ──
bad_shape = limiter.filter(torch.zeros(1, DIM + 3, device=env.device))
check("形状守卫：维度不对时保持上一步（不崩）", tuple(bad_shape.shape) == (1, DIM))

# ── 5) 集成：连续给超程动作，关节不能飞（停在内缩后的限位附近）──
limiter.reset()
for _ in range(90):
    env.step(limiter.filter(torch.full((1, DIM), 10.0, device=env.device)))
arm = env.scene["left_arm"]
joint_pos = arm.data.joint_pos
joint_pos = joint_pos.torch if hasattr(joint_pos, "torch") else joint_pos
pos = joint_pos[0].detach().cpu()
limits = arm.data.joint_pos_limits
limits = limits.torch if hasattr(limits, "torch") else limits
lim = limits[0].detach().cpu()
inside = bool(((pos >= lim[:, 0] - 1e-3) & (pos <= lim[:, 1] + 1e-3)).all())
check("集成：连续超程 90 步后关节仍在硬限位内", inside, f"joint_pos={[round(float(v), 3) for v in pos]}")

# ── 6) 声明式 clip 兜底（绕过 limiter 直接 env.step）──
print("\n  声明式 clip 字典（前 3 项）：", {k: tuple(round(x, 3) for x in v) for k, v in list(build_declarative_clip().items())[:3]})
install_declarative_clip(env)
term = env.action_manager.get_term("left_arm_action")
raw = torch.full((1, DIM), 10.0, device=env.device)
env.action_manager.process_action(raw)
processed = term.processed_actions[0].detach().cpu()
cfg_clip = term.cfg.clip
upper = torch.tensor([cfg_clip[j][1] for j in term._joint_names])
check(
    "声明式 clip：绕过包装器直接 env.step 也被裁",
    bool((processed <= upper + 1e-5).all()),
    f"processed={[round(float(v), 3) for v in processed]}",
)

print(f"\n===== 拦截统计：{limiter.stats}")
print("===== 结论：" + ("全部通过 ✅" if ok else "有失败项 ❌"))
env.close()
simulation_app.close()
raise SystemExit(0 if ok else 1)
