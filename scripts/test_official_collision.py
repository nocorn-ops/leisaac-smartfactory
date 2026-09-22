#!/usr/bin/env python
"""测试官方 SO101 在厨房场景中能否碰到橙子。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
from isaaclab.app import AppLauncher
import argparse
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium, leisaac, torch
from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
from isaaclab.managers import ObservationGroupCfg, ObservationTermCfg, SceneEntityCfg
from isaaclab.managers import TerminationTermCfg

# 用官方 SO101 替换 LeRobot 机器人
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG
from leisaac.assets.scenes.kitchen import KITCHEN_WITH_ORANGE_CFG
from leisaac.tasks.template.single_arm_env_cfg import SingleArmTaskEnvCfg

env_cfg = parse_env_cfg("LeIsaac-LeRobot-Kitchen-v0", device=args_cli.device, num_envs=1)
# 替换机器人为官方 SO101
env_cfg.scene.robot = SO101_FOLLOWER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
env_cfg.scene.robot.init_state.pos = (2.2, -0.61, 0.89)

env = gymnasium.make("LeIsaac-LeRobot-Kitchen-v0", cfg=env_cfg).unwrapped
env.reset()

print("官方 SO101 已加载到厨房场景。夹爪靠近橙子测试碰撞。")
print("按 Ctrl+C 退出")

import time
while simulation_app.is_running():
    env.sim.render()
    time.sleep(0.01)

env.close()
simulation_app.close()
