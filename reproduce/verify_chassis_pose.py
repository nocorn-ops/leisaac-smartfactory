#!/usr/bin/env python
"""验收：底盘"作业位"相关三件事（数采/导航都用得上）。

① `ChassisController.set_pose(x, y, yaw)` 能把机器人直接摆到作业位，**机械臂跟着一起走**
② `env.reset()` 会把 `Body` + 两条手臂写回**场景初始位姿**（= 按 `N`/`R` 换 demo 时
   "机器人闪回起点"的根因）
③ `ChassisController.apply_pose()` 能在 `env.reset()` 之后把作业位摆回来（这就是修复）

用法（宿主机、仓库根目录）::

    <ISAACSIM>/python.sh -u reproduce/verify_chassis_pose.py

通过标准：结尾 `✅ 底盘作业位/复位保持 全部通过`。
"""
import math
import os

from isaaclab.app import AppLauncher

# ★ 场地任务里声明了相机传感器；不开渲染会报 "Invalid object in Py_Graph"（踩过），
#   所以这里照 verify_bi_keyboard.py 一样把 enable_cameras 打开。
app_launcher = AppLauncher({"headless": True, "enable_cameras": True})
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import omni.usd  # noqa: E402

import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.assets.scenes.smart_factory_layout import WORK_ZONES  # noqa: E402
from leisaac.utils.chassis import ChassisController  # noqa: E402

TASK = "LeIsaac-SmartFactory-v0"
failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'✅' if ok else '❌'} {name}" + (f"  （{detail}）" if detail else ""))
    if not ok:
        failures.append(name)


def body_xy(env) -> tuple[float, float, float]:
    """底盘在世界系的位置 + 朝向（从 USD 世界变换读，绕开 IsaacLab 的四元数约定）。"""
    from pxr import UsdGeom

    prim = omni.usd.get_context().get_stage().GetPrimAtPath("/World/envs/env_0/Body")  # noqa: F821
    cache = UsdGeom.XformCache()
    t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    q = cache.GetLocalToWorldTransform(prim).ExtractRotationQuat()
    qx, qy, qz, qw = q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return float(t[0]), float(t[1]), float(yaw)


def arm_xy(env) -> tuple[float, float]:
    pos = env.scene["right_arm"].data.root_pos_w
    pos = pos.torch if hasattr(pos, "torch") else pos
    p = pos[0]
    return float(p[0]), float(p[1])


env_cfg = parse_env_cfg(TASK, device="cuda:0", num_envs=1)
env_cfg.use_teleop_device("bi-so101leader")
env_cfg.recorders = None
env = gym.make(TASK, cfg=env_cfg).unwrapped
env.reset()
for _ in range(10):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

chassis = ChassisController(env)
x0, y0, yaw0 = chassis.pose
print(f"[setup] 场景初始位姿 x={x0:+.3f} y={y0:+.3f} yaw={math.degrees(yaw0):+.1f}°")

sx, sy, syaw = WORK_ZONES["shelf"]
print(f"[setup] 目标作业位（收纳架前）x={sx:+.3f} y={sy:+.3f} yaw={math.degrees(syaw):+.1f}°")

print("\n===== ① set_pose 摆到作业位 =====")
arm_before = arm_xy(env)
chassis.set_pose(sx, sy, syaw)
for _ in range(5):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
bx, by, byaw = body_xy(env)
print(f"  底盘世界坐标 {bx:+.3f}, {by:+.3f} yaw={math.degrees(byaw):+.1f}°")
check("底盘停在作业位（±2cm / ±2°）",
      abs(bx - sx) < 0.02 and abs(by - sy) < 0.02 and abs(math.degrees(byaw - syaw)) < 2.0)
arm_after = arm_xy(env)
d_arm = math.hypot(arm_after[0] - arm_before[0], arm_after[1] - arm_before[1])
check("机械臂跟着底盘一起搬家", d_arm > 0.5, f"右臂根移动 {d_arm:.2f} m")
check("底盘 yaw 与要求一致（朝 −X = 面向架子开口）", abs(math.degrees(byaw) - 180.0) < 2.0,
      f"yaw={math.degrees(byaw):+.1f}°")

print("\n===== ② env.reset() 之后：底盘还在，但**机械臂会飞回起点** =====")
def arm_gap() -> float:
    """两条臂的根位置与底盘位置的距离（正常贴着底盘时 < 0.3 m）。"""
    bx2, by2, _ = body_xy(env)
    ax2, ay2 = arm_xy(env)
    return math.hypot(ax2 - bx2, ay2 - by2)

gap_before = arm_gap()
print(f"  reset 前：右臂根与底盘相距 {gap_before:.3f} m")
env.reset()
for _ in range(3):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
rx, ry, _ = body_xy(env)
gap_after = arm_gap()
print(f"  reset 后：底盘 {rx:+.3f}, {ry:+.3f}（离作业位 {math.hypot(rx - sx, ry - sy):.3f} m）")
print(f"  reset 后：右臂根与底盘相距 {gap_after:.3f} m")
check("确认 reset 会让机械臂脱离底盘（>1m）—— 所以必须补 apply_pose()", gap_after > 1.0,
      f"臂-底盘间距 {gap_after:.2f} m（正常 {gap_before:.2f} m）")

print("\n===== ③ apply_pose() 把机械臂摆回底盘上 =====")
chassis.apply_pose()
for _ in range(3):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)
ax, ay, ayaw = body_xy(env)
gap_fixed = arm_gap()
print(f"  apply_pose 后：底盘 {ax:+.3f}, {ay:+.3f} yaw={math.degrees(ayaw):+.1f}°，臂-底盘间距 {gap_fixed:.3f} m")
check("底盘仍在作业位（±2cm）", math.hypot(ax - sx, ay - sy) < 0.02,
      f"误差 {math.hypot(ax - sx, ay - sy) * 100:.1f} cm")
check("机械臂回到底盘上（间距回到 reset 前的水平）", gap_fixed < gap_before + 0.05,
      f"{gap_fixed:.3f} m vs 正常 {gap_before:.3f} m")

print("\n===== ④ 作业位在架子东侧、朝向开口（几何自检）=====")
check("收纳架前作业位在架子东侧（x 大于架子中心 2.27）", sx > 2.27, f"x={sx}")
check("朝向 −X（面向开口）", abs(math.degrees(syaw) - 180.0) < 1e-6, f"yaw={math.degrees(syaw):.1f}°")
check("四个作业位都有定义", set(WORK_ZONES) == {"home", "shelf", "transfer", "park"},
      f"{sorted(WORK_ZONES)}")

print("\n" + "=" * 64)
if failures:
    print("❌ 未通过：" + "、".join(failures))
else:
    print("✅ 底盘作业位/复位保持 全部通过")
print("=" * 64)

env.close()
simulation_app.close()
# simulation_app.close() 会直接结束进程，所以显式给退出码（详见 README §9.5 的提醒）
os._exit(1 if failures else 0)
