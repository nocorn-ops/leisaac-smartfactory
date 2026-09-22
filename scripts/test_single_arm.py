#!/usr/bin/env python
"""逐一测试 12 个关节：每个关节单独发 action=1.0，看是否移动到目标位置。"""
import multiprocessing
if multiprocessing.get_start_method() != "spawn":
    multiprocessing.set_start_method("spawn", force=True)
import argparse, torch
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(vars(args_cli))
simulation_app = app_launcher.app

import gymnasium, leisaac
from isaaclab_tasks.utils import parse_env_cfg

env_cfg = parse_env_cfg("LeIsaac-LeRobot-Kitchen-v0", device=args_cli.device, num_envs=1)
env = gymnasium.make("LeIsaac-LeRobot-Kitchen-v0", cfg=env_cfg).unwrapped
env.reset()
robot = env.scene["robot"]

# 动态获取 DOF 顺序（从 articulation，不硬编码）
all_joint_names = list(robot.joint_names)
print(f"=== Articulation DOF order: {all_joint_names} ===\n")

# 检查默认关节位和 offset
print(f"default_joint_pos: {[f'{v:.3f}' for v in robot.data.default_joint_pos[0].tolist()]}")
print(f"joint_pos_target (initial): {[f'{v:.3f}' for v in robot.data.joint_pos_target[0].tolist()]}")
# 检查 action term 的 offset
for term_name, term in env.action_manager._terms.items():
    if hasattr(term, '_offset'):
        print(f"\naction term '{term_name}':")
        print(f"  _offset shape: {term._offset.shape}")
        print(f"  _offset values: {[f'{v:.3f}' for v in term._offset[0].tolist()]}")

# 检查 JointPositionActionCfg 的 use_default_offset
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
for name, term in env.action_manager._terms.items():
    if isinstance(term, JointPositionAction):
        print(f"\n{name}: use_default_offset={term.cfg.use_default_offset}, scale={term.cfg.scale}")

joint_test = [
    (0, "left_shoulder_pan"),
    (1, "left_shoulder_lift"),
    (2, "left_elbow_flex"),
    (3, "left_wrist_flex"),
    (4, "left_wrist_roll"),
    (5, "left_gripper"),
    (6, "right_shoulder_pan"),
    (7, "right_shoulder_lift"),
    (8, "right_elbow_flex"),
    (9, "right_wrist_flex"),
    (10, "right_wrist_roll"),
    (11, "right_gripper"),
]

print("=== 逐个关节测试 (target=1.0, 60 steps each) ===\n")
print(f"{'ActionIdx':>10} {'Joint':>22} {'DOF':>4} {'Before':>8} {'After':>8} {'Moved?':>8}")
print("-" * 75)

for act_idx, joint_name in joint_test:
    dof = all_joint_names.index(joint_name) if joint_name in all_joint_names else -1
    if dof < 0:
        print(f"{act_idx:>10} {joint_name:>22} NOT FOUND!")
        continue
    env.reset()
    action = torch.zeros(1, 12, device=env.device)
    action[0, act_idx] = 1.0

    before = robot.data.joint_pos[0, dof].item()
    for _ in range(60):
        env.step(action)
    after = robot.data.joint_pos[0, dof].item()
    moved = abs(after - before) > 0.01
    print(f"{act_idx:>10} {joint_name:>22} {dof:>4} {before:>8.3f} {after:>8.3f} {'YES' if moved else 'NO':>8}")

env.close()
simulation_app.close()
