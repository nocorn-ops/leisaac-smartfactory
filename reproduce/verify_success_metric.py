#!/usr/bin/env python
"""验证厨房任务的**成功判据**：橘子在盘子外 → success=False；把橘子放进盘子 → success=True。

同时打印 `oranges_plate_distance` 进度指标。
用法: reproduce/verify_success_metric.py
"""
import dataclasses

from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True, enable_cameras=False)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab.sensors import TiledCameraCfg  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from leisaac.utils.env_utils import get_task_type  # noqa: E402
from leisaac.utils.eval_metrics import format_progress, get_success  # noqa: E402


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
env_cfg.enable_success()  # 默认关闭（保护采数据），评测时才开
env = gym.make("LeIsaac-LeRobot-Kitchen-v0", cfg=env_cfg).unwrapped
env.reset()
for _ in range(60):
    env.sim.step(render=False)
    env.scene.update(env.physics_dt)

ok = True


def check(name, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print(f"  {'✅' if cond else '❌'} {name}" + (f"  {detail}" if detail else ""))


print("\n===== 成功判据验证")
names = [t for t in env.termination_manager.active_terms]
print(f"  终止项 = {names}")
check("厨房任务现在有 success 终止项", "success" in names)
check("success 不是超时项", not bool(env.termination_manager.get_term_cfg("success").time_out))

# ── 1) 初始状态（橘子不在盘子里）→ 不应成功 ──
zero = torch.zeros(env.action_space.shape[-1], device=env.device).unsqueeze(0)
# 注意：不能靠 env.step() 来"看" success —— IsaacLab 3.0 的 step() 会在终止时**自动 reset**，
# 于是读到的永远是重置后的初始状态。这里手动 compute 终止项。
env.termination_manager.compute()
print(f"  初始：{format_progress(env)}")
check("初始状态 success=False", not bool(get_success(env)[0]))

# ── 2) 把两颗橘子放到盘子内（错开位置，避免相互重叠被弹开）→ 应成功 ──
plate = env.scene["Plate"]
plate_pos = plate.data.root_pos_w[0].clone()


def put(name, offset):
    obj = env.scene[name]
    before = obj.data.root_pos_w[0].detach().clone()
    # 用 write_root_state_to_sim（位姿+速度一起写）：只写 pose 时，休眠中的刚体在物理步里
    # 可能不接受这次"传送"，读回的 root_pos_w 是写入缓冲、而物理状态没变。
    state = torch.cat(
        [
            plate_pos + torch.tensor(offset, device=env.device),
            obj.data.root_quat_w[0],
            torch.zeros(6, device=env.device),
        ]
    ).unsqueeze(0)
    obj.write_root_state_to_sim(state)
    env.scene.update(env.physics_dt)
    after = obj.data.root_pos_w[0].detach().clone()
    print(f"    {name}: 写入 {[round(float(v), 3) for v in state[0][:3]]} | "
          f"写前 {[round(float(v), 3) for v in before[:3]]} → 写后 {[round(float(v), 3) for v in after[:3]]}")
    return after


put("Orange002", (0.03, 0.0, 0.02))
put("Orange003", (-0.03, 0.0, 0.02))
env.scene.update(env.physics_dt)
env.termination_manager.compute()
print(f"  放进盘子后：{format_progress(env)}")
check("把橘子放进盘子后 success=True", bool(get_success(env)[0]))

# ── 3) 只放一颗 → 不应成功 ──
obj = env.scene["Orange003"]
far = torch.cat(
    [
        plate_pos + torch.tensor([0.5, 0.0, 0.01], device=env.device),
        obj.data.root_quat_w[0],
        torch.zeros(6, device=env.device),
    ]
).unsqueeze(0)
obj.write_root_state_to_sim(far)
env.scene.update(env.physics_dt)
env.termination_manager.compute()
print(f"  只放一颗：{format_progress(env)}")
check("只放一颗时 success=False（要求全部）", not bool(get_success(env)[0]))

print("\n===== 结论：" + ("成功判据工作正常 ✅" if ok else "有问题 ❌"))
env.close()
simulation_app.close()
raise SystemExit(0 if ok else 1)
