#!/usr/bin/env python
"""LeIsaac 任务级兼容性审计（不开窗，纯配置层）。

对每个注册的 LeIsaac-* 任务调用 parse_env_cfg（会执行 EnvCfg.__post_init__），
逐个记录成功/失败与失败原因 —— 用来定位 3.0 迁移还差哪些任务。
配置层过了不代表运行层一定过（Direct 环境、资产缺失等要单独验）。
"""
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import leisaac  # noqa: E402, F401
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

TASKS = sorted(k for k in gym.envs.registry.keys() if k.startswith("LeIsaac"))

print(f"[audit] 共注册 {len(TASKS)} 个 LeIsaac 任务")
ok, bad = [], []
for task in TASKS:
    try:
        parse_env_cfg(task, device="cuda:0", num_envs=1)
        ok.append(task)
        print(f"[audit] OK   {task}", flush=True)
    except Exception as e:  # noqa: BLE001
        bad.append((task, f"{type(e).__name__}: {e}"))
        print(f"[audit] FAIL {task} -> {type(e).__name__}: {e}", flush=True)

print(f"\n[audit] 配置层通过 {len(ok)}/{len(TASKS)}")
for task, err in bad:
    print(f"[audit]   失败: {task} -> {err[:160]}")

simulation_app.close()
