#!/usr/bin/env python
"""查看任务场景，不需要控制器。关闭窗口退出。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="查看 LeIsaac 任务场景。")
parser.add_argument("--task", type=str, default=None, help="任务名")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=None)
# IsaacLab 3.0 起 --headless / --enable_cameras 不再是 CLI 参数（改成 AppLauncher 的配置键），
# 且这两个字段名不能再被 argparse 占用（AppLauncher 的冲突检查按 argparse 的 dest 判定）。
# 这里用别名 dest 保留原来的命令行写法，解析后再写回 AppLauncher 认识的键。
parser.add_argument(
    "--enable_cameras", dest="enable_cameras_flag", action="store_true", default=False, help="Enable camera sensors."
)
parser.add_argument(
    "--headless", dest="headless_flag", action="store_true", default=False, help="Run without a window."
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = bool(args_cli.enable_cameras_flag)
args_cli.headless = True if args_cli.headless_flag else None  # None = 交给 AppLauncher 按 HEADLESS 环境变量/默认值决定

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import time, gymnasium as gym
import leisaac  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg
from leisaac.utils.env_utils import get_task_type

# 创建环境
env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
task_type = get_task_type(args_cli.task)
env_cfg.use_teleop_device(task_type)
env_cfg.recorders = None
if args_cli.seed is not None:
    env_cfg.seed = args_cli.seed

print(f"正在加载: {args_cli.task}")
env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
env.reset()

print("场景已就绪。关闭窗口退出。")
while simulation_app.is_running():
    env.sim.render()
    time.sleep(0.01)

env.close()
simulation_app.close()
