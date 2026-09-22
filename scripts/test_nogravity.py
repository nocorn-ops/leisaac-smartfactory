#!/usr/bin/env python
"""测试：关闭重力后 shoulder_pan 是否受控。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)

from isaaclab.app import AppLauncher
import argparse, torch

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="LeIsaac-SmartFactory-v0")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium, leisaac
from isaaclab_tasks.utils import parse_env_cfg

env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)

# Test 1: 只关闭自碰撞，保持重力
env_cfg.scene.robot.spawn.rigid_props.disable_gravity = False
env_cfg.scene.robot.spawn.articulation_props.enabled_self_collisions = False

env1 = gymnasium.make(args_cli.task, cfg=env_cfg).unwrapped
obs, _ = env1.reset()
robot = env1.scene["robot"]

action = torch.zeros(1, 12, device=env1.device)
action[0, 0] = 1.0; action[0, 6] = -1.0

print("=== 测试1: 重力=ON, 自碰撞=OFF ===")
print(f"初始: left_sp={robot.data.joint_pos[0,1]:.3f} right_sp={robot.data.joint_pos[0,2]:.3f}")
for i in range(200):
    env1.step(action)
print(f"最终: left_sp={robot.data.joint_pos[0,1]:.3f} (target=1.0)  right_sp={robot.data.joint_pos[0,2]:.3f} (target=-1.0)")
if robot.data.joint_pos[0,1] > 0.5:
    print(">> 自碰撞是根因！")
else:
    print(">> 自碰撞不是根因，重力才是。")
env1.close()

# Test 2: 只关闭重力，保持自碰撞
env_cfg2 = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1)
env_cfg2.scene.robot.spawn.rigid_props.disable_gravity = True
env_cfg2.scene.robot.spawn.articulation_props.enabled_self_collisions = True
env2 = gymnasium.make(args_cli.task, cfg=env_cfg2).unwrapped
obs, _ = env2.reset()
robot2 = env2.scene["robot"]
print(f"\n=== 测试2: 重力=OFF, 自碰撞=ON ===")
print(f"初始: left_sp={robot2.data.joint_pos[0,1]:.3f} right_sp={robot2.data.joint_pos[0,2]:.3f}")
for i in range(200):
    env2.step(action)
print(f"最终: left_sp={robot2.data.joint_pos[0,1]:.3f} (target=1.0)  right_sp={robot2.data.joint_pos[0,2]:.3f} (target=-1.0)")
if robot2.data.joint_pos[0,1] > 0.5:
    print(">> 重力是根因！自碰撞无影响。")
else:
    print(">> 重力和自碰撞各自独立造成问题。")
env2.close()
simulation_app.close()
