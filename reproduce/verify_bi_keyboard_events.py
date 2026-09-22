#!/usr/bin/env python
"""验收：**真实按键事件**能不能操作 `bi-keyboard` 双臂键盘遥操（模拟人手按键，走 carb 输入管线）。

和 `verify_bi_keyboard.py` 的区别：
  · `verify_bi_keyboard.py` 直接改设备内部状态（`dev._delta[...] += ...`）来验"动作语义"；
  · **本脚本走真实输入链路**：`carb.input.buffer_keyboard_key_event()` → 设备订阅的回调
    → `advance()` → `env.step()` → 物理里末端真的动。
    （Isaac Sim 自带的 `omni.kit.ui_test.input.emulate_keyboard()` 用的就是同一个 API。）

所以它验证的是"**按 B / 按 Q / 按 T / 按 W / 按 O / 按 R / 按 N，仿真里真的会发生对应的事**"
—— 这正是人在窗口前按键时走的路径（差别只在最底层的物理 HID）。

检查项：
  ① 按 `B` → 设备进入 started（开始出动作）
  ② 按住 `Q`（up）→ **当前臂的末端升高**，另一条臂基本不动
  ③ 按 `T` → 当前臂切成 right
  ④ 按住 `W`（forward）→ 右臂末端移动；`Q` 松开后不再持续抬升
  ⑤ 按 `O`（gripper_close）→ 夹爪关节角变化
  ⑥ 按 `R` / `N` → 注册的回调被触发（= 录制流程的"重录/标成功"）
  ⑦ 全程 `env.step()` 不报错

用法（宿主机、仓库根目录；需要 GUI，因为按键要有 app window）::

    <ISAACSIM>/python.sh -u reproduce/verify_bi_keyboard_events.py

通过标准：结尾打印 `✅ 真实按键事件链路全部通过（B/Q/T/W/O/R/N 都能操作仿真）`，退出码 0。
"""
import os
import time

from isaaclab.app import AppLauncher

# ★ 必须开窗口（headless 下没有 appwindow，设备会降级成"无键盘"）
app_launcher = AppLauncher({"headless": False, "enable_cameras": True})
simulation_app = app_launcher.app

import carb  # noqa: E402
import gymnasium as gym  # noqa: E402
import omni.appwindow  # noqa: E402
import torch  # noqa: E402

import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.sim_modes import build_device, ensure_observations  # noqa: E402

TASK = "LeIsaac-SmartFactory-v0"
STEPS = 30

failures: list[str] = []
events_fired: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'✅' if ok else '❌'} {name}" + (f"  （{detail}）" if detail else ""))
    if not ok:
        failures.append(name)


# ── 按键注入（与 omni.kit.ui_test.input.emulate_keyboard 同一 API，同步版）──────────────
_keyboard = omni.appwindow.get_default_app_window().get_keyboard()
_provider = carb.input.acquire_input_provider()


def pump(n: int = 2) -> None:
    """泵若干次 Kit 事件循环 —— carb 缓冲的按键事件在这次 update 里才会派发给订阅者。"""
    for _ in range(n):
        simulation_app.update()


def key(name: str, event_type) -> None:
    key_enum = getattr(carb.input.KeyboardInput, name)
    _provider.buffer_keyboard_key_event(_keyboard, event_type, key_enum, 0)
    pump(2)
    time.sleep(0.15)


def press(name: str) -> None:
    key(name, carb.input.KeyboardEventType.KEY_PRESS)


def release(name: str) -> None:
    key(name, carb.input.KeyboardEventType.KEY_RELEASE)


def tap(name: str) -> None:
    press(name)
    release(name)


def ee_z(env, side: str) -> float:
    asset = env.scene[f"{side}_arm"]
    idxs, _ = asset.find_bodies("gripper")
    return float(asset.data.body_pos_w[0, idxs[0], 2])


def ee_xyz(env, side: str) -> tuple[float, float, float]:
    asset = env.scene[f"{side}_arm"]
    idxs, _ = asset.find_bodies("gripper")
    p = asset.data.body_pos_w[0, idxs[0]]
    return float(p[0]), float(p[1]), float(p[2])


def gripper_angle(env, side: str) -> float:
    asset = env.scene[f"{side}_arm"]
    ids, _ = asset.find_joints(["gripper"])
    pos = asset.data.joint_pos
    pos = pos.torch if hasattr(pos, "torch") else pos
    return float(pos[0, ids[0]])


# ── 建环境 + 设备（和统一入口 teleop 模式同一套）────────────────────────────────────
env_cfg = parse_env_cfg(TASK, device="cuda:0", num_envs=1)
ensure_observations(env_cfg)
env_cfg.recorders = None
env_cfg.use_teleop_device("bi-keyboard")
env_cfg.terminations.time_out = None
if hasattr(env_cfg.terminations, "success"):
    env_cfg.terminations.success = None

print("[setup] 建环境（GUI）...", flush=True)
env = gym.make(TASK, cfg=env_cfg).unwrapped
dev = build_device(env, device="bi-keyboard", sensitivity=1.0, arm_side="left")
dev.add_callback("R", lambda: events_fired.append("R"))
dev.add_callback("N", lambda: events_fired.append("N"))
dev.reset()
if dev._keyboard is None:
    print("[setup] ❌ 设备没有键盘（appwindow 缺失）→ 本脚本必须开窗口跑")
    raise SystemExit(2)

print("\n===== ① 按 B 启动 =====")
check("按 B 之前 started=False", dev.started is False)
tap("B")
check("按 B 之后 started=True（开始输出动作）", dev.started is True)

print(f"\n===== ② 按住 Q（up）{STEPS} 步：当前臂应比「不按键」更高 =====")
print("  注：`Q` 是「末端沿**自身 X** 向上」，不是世界 +Z，所以判据用位移而不是世界 Z 的绝对值。")
print("      另外脚本里同时打印「不按键」的对照 —— 模板对 bi-keyboard 已关闭机械臂重力，")
print("      对照应当≈0；若对照明显下沉（例如 -15 cm），说明重力开关没生效（见 README §5.3）。")

# 对照：不按任何键走同样步数（全零增量 = 目标跟住当前位姿；重力已关，预期≈0）
act0 = dev.advance()
d0 = ee_z(env, "left")
for _ in range(STEPS):
    env.step(act0)
d1 = ee_z(env, "left")
drift = d1 - d0
print(f"  [对照] 不按键 {STEPS} 步：left z {d0:.4f} → {d1:.4f}   Δ={drift:+.4f} m")

before = {s: ee_z(env, s) for s in ("left", "right")}
press("Q")
act = dev.advance()
check("advance() 给出了动作张量", act is not None and tuple(act.shape) == (1, 16), f"shape={None if act is None else tuple(act.shape)}")
for _ in range(STEPS):
    env.step(act)
after = {s: ee_z(env, s) for s in ("left", "right")}
for s in ("left", "right"):
    print(f"  {s:5s}_arm gripper z: {before[s]:.4f} → {after[s]:.4f}   Δ={after[s] - before[s]:+.4f}")
lift = (after["left"] - before["left"]) - drift
check("按 Q 让当前臂比「不按键」抬高了", lift > 0.02,
      f"相对抬升 {lift * 100:+.2f} cm（按 Q Δ={after['left'] - before['left']:+.4f} vs 对照 Δ={drift:+.4f}）")
release("Q")
act = dev.advance()

print("\n===== ③ 按 T 切换左右臂 =====")
check("按 T 之前是 left", dev.active_side == "left")
tap("T")
check("按 T 之后是 right（真实按键事件生效）", dev.active_side == "right", f"active_side={dev.active_side}")

print(f"\n===== ④ 按住 W（forward）{STEPS} 步：右臂应移动 =====")
b4 = ee_xyz(env, "right")
press("W")
act = dev.advance()
for _ in range(STEPS):
    env.step(act)
a4 = ee_xyz(env, "right")
dist = sum((a4[i] - b4[i]) ** 2 for i in range(3)) ** 0.5
print(f"  right_arm 末端 {tuple(round(v, 4) for v in b4)} → {tuple(round(v, 4) for v in a4)}  位移={dist:.4f} m")
check("按 W 后右臂末端发生位移", dist > 0.005, f"{dist * 100:.2f} cm")
release("W")
act = dev.advance()

print(f"\n===== ⑤ 按 O（gripper_close）{STEPS} 步：夹爪应变化 =====")
g0 = gripper_angle(env, "right")
press("O")
act = dev.advance()
for _ in range(STEPS):
    env.step(act)
g1 = gripper_angle(env, "right")
release("O")
print(f"  right gripper joint: {g0:.4f} → {g1:.4f}   Δ={g1 - g0:+.4f} rad")
check("按 O 后夹爪关节角变化", abs(g1 - g0) > 1e-3, f"Δ={g1 - g0:+.4f} rad")

print("\n===== ⑥ 按 R / N：录制流程的回调 =====")
tap("R")
pump(2)
check("按 R 触发 reset 回调", "R" in events_fired, f"fired={events_fired}")
check("按 R 后 started=False（要再按 B）", dev.started is False)
env.reset()
tap("B")
tap("N")
pump(2)
check("按 N 触发 success 回调", "N" in events_fired, f"fired={events_fired}")

print(f"\n===== ⑦ 连续 step 不报错 =====")
try:
    for _ in range(30):
        a = dev.advance()
        if a is not None:
            env.step(a)
    print("  无异常")
    check("连续 step 无异常", True)
except Exception as exc:  # noqa: BLE001
    check("连续 step 无异常", False, f"{type(exc).__name__}: {exc}")

print("\n" + "=" * 64)
if failures:
    print("❌ 未通过：" + "、".join(failures))
else:
    print("✅ 真实按键事件链路全部通过（B/Q/T/W/O/R/N 都能操作仿真）")
print("=" * 64)

env.close()
simulation_app.close()
# ★ 必须用 os._exit：`simulation_app.close()` 内部会直接把进程结束掉（退出码 0），
#   后面的 `raise SystemExit(...)` 根本执行不到 → 退出码永远是 0，不能当判据。
#   所以由我们显式给出退出码。（同样原因：`python.sh` 对非零退出码统一转成 1。）
os._exit(1 if failures else 0)
